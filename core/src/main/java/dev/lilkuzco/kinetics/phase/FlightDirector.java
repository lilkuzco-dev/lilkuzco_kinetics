package dev.lilkuzco.kinetics.phase;

import dev.lilkuzco.kinetics.body.KineticBody;
import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.env.Environment;
import dev.lilkuzco.kinetics.event.EventSink;
import dev.lilkuzco.kinetics.event.KineticEvent;
import dev.lilkuzco.kinetics.guidance.LoftProfile;
import dev.lilkuzco.kinetics.guidance.ProportionalNavigation;
import dev.lilkuzco.kinetics.guidance.Seeker;
import dev.lilkuzco.kinetics.guidance.Target;
import dev.lilkuzco.kinetics.integrate.ControlCommand;
import dev.lilkuzco.kinetics.integrate.Integrator;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.profile.EngineFrame;
import dev.lilkuzco.kinetics.profile.Profile;
import dev.lilkuzco.kinetics.propulsion.GravityTurn;
import dev.lilkuzco.kinetics.propulsion.PoweredDescent;
import dev.lilkuzco.kinetics.sensors.Countermeasures;

import java.util.List;

/**
 * Drives one body through the flight-phase state machine (Section 2).
 *
 * <p>{@link Integrator} knows physics and nothing else; this knows what phase the body is in,
 * what that phase implies for steering and thrust, and when to move on. Keeping them apart is
 * what makes the state machine auditable - every transition in the flight comes from a named
 * branch here and is logged with its reason, rather than emerging from conditions scattered
 * through the force model.
 *
 * <p>All three mission kinds share the same machine. A mortar shell is a launch vehicle that
 * never leaves DESCENT; an interceptor is one whose terminal phase has a seeker. There is no
 * separate code path per weapon type, which is what lets warfront, naval and cosmos all consume
 * this without kinetics knowing what any of them are.
 */
public final class FlightDirector {

    /** What this body is trying to do. Selects the guidance chain, not the physics. */
    public enum Mission {
        /** Unguided. Fired and forgotten; drag and gravity do the rest. */
        BALLISTIC,
        /** Seeker-guided: boost, midcourse PN, terminal. */
        GUIDED,
        /** Reaching orbit: gravity turn, staging, insertion or an honest ballistic failure. */
        LAUNCH,
        /**
         * Arriving intact: coast, then a retro-burn sized to the vehicle's own thrust (RD6).
         *
         * <p>Its own mission rather than a flag on LAUNCH because it is the opposite problem.
         * A launch spends propellant to gain energy against a budget; a landing spends propellant
         * to shed energy against a deadline, and the deadline is the ground.
         */
        LANDING
    }

    private final Constants k;
    private final Integrator integrator;
    private final Environment env;
    private final KineticBody body;
    private final Mission mission;
    private final EngineFrame engineFrame;

    private final Seeker seeker;
    private final ProportionalNavigation pn;
    private final GravityTurn gravityTurn;
    private final PoweredDescent poweredDescent;
    private LoftProfile loft;

    private final double karmanAltitude;
    private final double minTwr;
    private final double reentryFraction;
    private final double fuseRadius;
    private final double terminalRangeFraction;
    private final long seed;

    private Vec3 downrange = new Vec3(1, 0, 0);
    private boolean insertionAttempted;
    private double lastInsertionDeltaV;

    // Closest-approach tracking for the proximity fuse.
    //
    // Sampling the range once per tick is not good enough and the numbers say why: an
    // interceptor closing at 460 m/s covers 23 m in a single tick, so a perfect intercept and a
    // 23 m miss produce the same sequence of sampled ranges. The fuse therefore solves for the
    // closest approach WITHIN the tick, treating both bodies as moving linearly across it - the
    // same calculation a real miss-distance analysis does.
    private double previousRange = Double.MAX_VALUE;
    private double closestRange = Double.MAX_VALUE;
    private Vec3 previousBodyPosition;

    public FlightDirector(Constants k, Environment env, KineticBody body, Mission mission,
                          Integrator integrator, long seed) {
        this.k = k;
        this.env = env;
        this.body = body;
        this.mission = mission;
        this.integrator = integrator;
        this.seed = seed;
        this.engineFrame = EngineFrame.of(k);

        Profile profile = body.profile();
        this.seeker = profile.seeker().isPresent()
                ? new Seeker(body.id(), profile.seeker(), k) : null;
        this.pn = new ProportionalNavigation(k, profile.seeker());
        this.gravityTurn = mission == Mission.LAUNCH ? GravityTurn.standard(k) : null;
        this.poweredDescent = mission == Mission.LANDING ? new PoweredDescent(k) : null;
        this.loft = null;

        this.karmanAltitude = k.d("atmosphere.karman_altitude_game");
        this.minTwr = k.d("propulsion.min_liftoff_twr");
        this.reentryFraction = k.d("reentry.reentry_phase_heating_fraction");
        this.fuseRadius = k.d("guidance.proximity_fuse_radius_default");
        this.terminalRangeFraction = k.d("guidance.terminal_phase_range_fraction");
    }

    /**
     * Fly a lofted trajectory (RC4): climb first, then blend smoothly into the terminal law.
     * Buys range by spending most of the flight in thin air.
     */
    public FlightDirector withLoft(LoftProfile profile) {
        this.loft = profile;
        return this;
    }

    /** Set the horizontal direction a launch vehicle flies its gravity turn toward. */
    public FlightDirector downrange(Vec3 heading) {
        Vec3 h = new Vec3(heading.x(), 0.0, heading.z());
        this.downrange = h.lengthSq() < 1e-12 ? new Vec3(1, 0, 0) : h.normalized();
        return this;
    }

    public Seeker seeker() { return seeker; }

    public double lastInsertionDeltaV() { return lastInsertionDeltaV; }

    /** Closest approach to the target so far, m. {@code MAX_VALUE} if never in TERMINAL. */
    public double closestApproach() { return closestRange; }

    /**
     * Advance one tick: decide the phase, decide the command, integrate, then react to what
     * happened.
     */
    public Integrator.StepResult tick(double worldTime, double dt, Target target,
                                      List<Countermeasures.Decoy> decoys, EventSink events) {
        if (!body.phase().isInWorld()) return null;

        preStepTransitions(worldTime, target, events);
        if (!body.phase().isInWorld()) return null;

        ControlCommand command = commandFor(dt, target, decoys, events);
        Integrator.StepResult result = integrator.step(body, env, command, worldTime, dt, events);

        postStepTransitions(result, target, dt, events);
        return result;
    }

    // ---- transitions ------------------------------------------------------

    private void preStepTransitions(double worldTime, Target target, EventSink events) {
        double altitude = env.altitudeOf(body.position().y());

        switch (body.phase()) {
            case RAIL -> {
                // RD5: the thrust-to-weight gate, evaluated with SEA-LEVEL thrust. A vehicle
                // that fails it does not ascend slowly - it does not move, and saying so at
                // ignition is more useful than letting it sit on the pad burning fuel.
                double twr = body.profile().liftoffTwr(env.gravity(), engineFrame);
                if (twr <= minTwr) {
                    events.accept(new KineticEvent.LiftoffFailure(
                            body.id(), body.age(), twr, minTwr));
                    body.phases().transition(FlightPhase.TERMINATED, body.age(),
                            String.format("liftoff failed: T/W %.3f <= %.2f", twr, minTwr), events);
                    return;
                }
                body.phases().transition(FlightPhase.BOOST, body.age(),
                        String.format("ignition at T/W %.3f", twr), events);
            }
            case BOOST -> {
                // Check the terminal handover BEFORE burnout: a missile can arrive while still
                // burning, and if the endgame had to wait for the motor to quit the fuse would
                // still be safed as it flew past.
                if (enteredTerminal(target, events)) return;
                if (!body.hasFuel()) {
                    boolean moreStages = body.stageIndex() + 1 < body.profile().stages().size();
                    if (moreStages) {
                        body.phases().transition(FlightPhase.STAGING, body.age(),
                                "stage " + body.stageIndex() + " burnout", events);
                    } else {
                        body.advanceStage(events);   // shed the last stage's structure
                        onFinalBurnout(worldTime, altitude, events);
                    }
                }
            }
            case STAGING -> {
                if (body.advanceStage(events)) {
                    body.phases().transition(FlightPhase.BOOST, body.age(),
                            "stage " + body.stageIndex() + " ignition", events);
                } else {
                    onFinalBurnout(worldTime, altitude, events);
                }
            }
            case MIDCOURSE -> {
                if (enteredTerminal(target, events)) {
                    return;
                } else if (body.velocity().y() < 0.0 && altitude < karmanAltitude
                        && mission != Mission.GUIDED) {
                    // Falling and no longer closing: this is a descent now.
                    body.phases().transition(FlightPhase.DESCENT, body.age(),
                            "past apex, descending", events);
                }
            }
            case TERMINAL -> {
                // A missile that burns out during the endgame still sheds its spent structure,
                // so the mass ledger stays closed (I4) right through to the fuse.
                if (!body.hasFuel() && body.hasStagesRemaining()) body.advanceStage(events);
            }
            case DESCENT -> {
                if (beginLandingBurn(altitude, events)) return;
                double threshold = body.profile().airframe().overheatThreshold() * reentryFraction;
                if (body.heatingRate() > threshold) {
                    body.phases().transition(FlightPhase.REENTRY, body.age(),
                            String.format("heating %.4g W/m2 above the %.4g W/m2 reentry threshold",
                                    body.heatingRate(), threshold), events);
                }
            }
            case LANDING -> {
                // Out of propellant mid-burn. It is a falling object again, and it will arrive at
                // whatever speed it had - which is the honest outcome of an under-fuelled lander.
                if (!body.hasFuel()) {
                    if (body.hasStagesRemaining()) {
                        body.advanceStage(events);
                    } else {
                        body.phases().transition(FlightPhase.DESCENT, body.age(),
                                String.format("landing burn ran dry at %.1f m and %.1f m/s",
                                        altitude, body.velocity().length()), events);
                    }
                }
            }
            case REENTRY -> {
                double threshold = body.profile().airframe().overheatThreshold() * reentryFraction;
                if (body.heatingRate() < threshold * 0.5) {
                    body.phases().transition(FlightPhase.DESCENT, body.age(),
                            "heating subsided", events);
                }
            }
            default -> { }
        }
    }

    /**
     * Hand over to the endgame if the target is inside the terminal range. TERMINAL is where the
     * fuse arms; before that a near miss means nothing.
     *
     * @return true if the transition happened
     */
    private boolean enteredTerminal(Target target, EventSink events) {
        if (mission != Mission.GUIDED || target == null || seeker == null) return false;
        double range = target.position().sub(body.position()).length();
        double handover = body.profile().seeker().maxRange() * terminalRangeFraction;
        if (range > handover) return false;
        return body.phases().transition(FlightPhase.TERMINAL, body.age(),
                String.format("range %.1f m inside the %.1f m terminal handover", range, handover),
                events);
    }

    /** All propellant spent. Either this made orbit, or it is now a falling object. */
    private void onFinalBurnout(double worldTime, double altitude, EventSink events) {
        if (mission == Mission.LAUNCH && altitude >= karmanAltitude && !insertionAttempted) {
            insertionAttempted = true;
            lastInsertionDeltaV = body.achievedDeltaV();
            // The registry makes the actual call; the director only reports which way it went.
            body.phases().transition(FlightPhase.MIDCOURSE, body.age(),
                    String.format("final burnout above the Karman line with %.1f m/s achieved",
                            body.achievedDeltaV()), events);
            return;
        }
        body.phases().transition(FlightPhase.MIDCOURSE, body.age(),
                String.format("final burnout at %.1f m with %.1f m/s achieved",
                        altitude, body.achievedDeltaV()), events);
    }

    private void postStepTransitions(Integrator.StepResult result, Target target,
                                     double tickSeconds, EventSink events) {
        if (result == null) return;
        double altitude = env.altitudeOf(body.position().y());

        // Proximity fuse, armed only in TERMINAL. It fires at closest approach - detected by the
        // range starting to grow again - rather than at a threshold, because a fast body can
        // cross the whole fuse radius inside one tick and never be sampled inside it.
        if (body.phase() == FlightPhase.TERMINAL && target != null) {
            double range = target.position().sub(body.position()).length();
            double sweptMiss = previousBodyPosition == null ? range
                    : closestApproachWithinTick(previousBodyPosition, body.position(),
                            target, tickSeconds);
            if (sweptMiss < closestRange) closestRange = sweptMiss;
            previousBodyPosition = body.position();

            if (range > previousRange && closestRange <= fuseRadius) {
                events.accept(new KineticEvent.Proximity(body.id(), body.age(), body.position(),
                        body.velocity(), body.mass(), target.id(), closestRange));
                body.phases().transition(FlightPhase.TERMINATED, body.age(),
                        String.format("proximity fuse at %.3f m", closestRange), events);
                return;
            }
            previousRange = range;
        }

        // RB6: dynamic pressure past the airframe limit. Reported once; kinetics does not
        // decide what breakup looks like (I10).
        double qMax = body.profile().airframe().qMaxPa();
        if (!body.structuralLimitFlagged() && result.maxDynamicPressure() > qMax) {
            body.flagStructuralLimit();
            events.accept(new KineticEvent.StructuralLimit(body.id(), body.age(),
                    body.position(), result.maxDynamicPressure(), qMax));
        }

        // RE7: heating past the profile threshold. Also reported, never applied.
        double heatLimit = body.profile().airframe().overheatThreshold();
        if (!body.overheatFlagged() && result.peakHeating() > heatLimit) {
            body.flagOverheat();
            events.accept(new KineticEvent.ReentryOverheat(body.id(), body.age(),
                    result.peakHeating(), heatLimit, altitude));
        }

        // Chutes, in declared order, gated on q (RB6).
        if (body.phase() == FlightPhase.DESCENT || body.phase() == FlightPhase.REENTRY) {
            body.tryDeployNextChute(altitude, body.dynamicPressure(), events);
        }

        if (result.collided()) {
            // Arriving under a burn is a landing, not a crash. Leaving LANDING out of this list
            // meant a lander that had cancelled its velocity perfectly and settled at 0.4 m/s was
            // still recorded as TERMINATED - the flight succeeded and the outcome said otherwise.
            FlightPhase next = body.phase() == FlightPhase.DESCENT
                    || body.phase() == FlightPhase.REENTRY
                    || body.phase() == FlightPhase.LANDING
                    ? FlightPhase.LANDED : FlightPhase.TERMINATED;
            body.phases().transition(next, body.age(), "impact", events);
        }
    }

    /**
     * Minimum separation during a tick, treating both bodies as moving linearly across it.
     *
     * <p>With relative start offset {@code r0} and relative displacement {@code dr}, the
     * separation is minimised at {@code t* = -(r0·dr)/|dr|²}, clamped to the tick. A negative
     * {@code t*} means they were already separating at the start; past 1 means they are still
     * closing at the end, and the next tick will catch it.
     */
    private static double closestApproachWithinTick(Vec3 bodyStart, Vec3 bodyEnd,
                                                    Target target, double dt) {
        Vec3 targetStart = target.position();
        Vec3 targetEnd = targetStart.add(target.velocity().scale(dt));
        Vec3 r0 = bodyStart.sub(targetStart);
        Vec3 dr = bodyEnd.sub(bodyStart).sub(targetEnd.sub(targetStart));

        double denom = dr.lengthSq();
        if (denom < 1e-18) return r0.length();
        double t = -r0.dot(dr) / denom;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        return r0.add(dr.scale(t)).length();
    }

    // ---- commands ---------------------------------------------------------

    private ControlCommand commandFor(double dt, Target target,
                                      List<Countermeasures.Decoy> decoys, EventSink events) {
        return switch (body.phase()) {
            case BOOST -> boostCommand(dt, target, decoys, events);
            case MIDCOURSE, TERMINAL -> guidedCommand(dt, target, decoys, events);
            case LANDING -> landingCommand();
            case STAGING -> ControlCommand.coast();
            default -> ControlCommand.coast();   // DESCENT, REENTRY: weathervane into the airflow
        };
    }

    /**
     * Should the retro-burn start? Asked every tick during the fall, answered by the closed form.
     *
     * <p>Only for a LANDING mission with propellant left: an ordinary falling booster is not a
     * lander, and a lander with dry tanks is not either.
     */
    private boolean beginLandingBurn(double altitude, EventSink events) {
        if (poweredDescent == null || !body.hasFuel()) return false;
        PoweredDescent.Command command = descentCommand(altitude);
        if (!command.burn()) return false;
        return body.phases().transition(FlightPhase.LANDING, body.age(),
                String.format("retro-burn at %.1f m, %.1f m/s, burn height %.1f m",
                        altitude, body.velocity().length(), command.burnAltitude()), events);
    }

    /** The descent law's verdict at this altitude, using the thrust actually available there. */
    private PoweredDescent.Command descentCommand(double altitude) {
        var stage = body.currentStage();
        double thrust = stage == null ? 0.0
                : stage.effectiveThrust(env.pressureRatioAt(body.position().y()), engineFrame);
        return poweredDescent.decide(altitude, body.velocity(), thrust, body.mass(),
                env.gravity());
    }

    /** Retrograde at full throttle, or coast once slow enough to settle. */
    private ControlCommand landingCommand() {
        PoweredDescent.Command command = descentCommand(env.altitudeOf(body.position().y()));
        return command.burn()
                ? ControlCommand.pointAt(command.direction(), command.throttle())
                : ControlCommand.coast();
    }

    private ControlCommand boostCommand(double dt, Target target,
                                        List<Countermeasures.Decoy> decoys, EventSink events) {
        if (mission == Mission.LAUNCH && gravityTurn != null) {
            double altitude = env.altitudeOf(body.position().y());
            Vec3 direction = gravityTurn.desiredDirection(altitude, downrange, body.velocity(),
                    body.dynamicPressure(), body.profile().airframe().qMaxPa());
            return ControlCommand.pointAt(direction, 1.0);
        }
        if (mission == Mission.GUIDED && target != null) {
            // Give the seeker its look FIRST, before deciding how to steer. During an alignment
            // turn the nose is pointed straight at the target, which is the best look angle the
            // head will ever get - skipping the update there means the missile finishes its
            // turn with no lock, no line-of-sight history, and PN starting cold on a target it
            // is already crossing. That produced a 113 m miss until it was fixed.
            Seeker.Fix fix = updateSeeker(dt, target, decoys, events);

            // RC3: off-boresight launches still get an alignment phase before PN is allowed to
            // steer, because PN fed geometry outside its derivation commands a turn it cannot
            // make and throws the missile's energy away doing it.
            if (pn.needsBoresightAlignment(body.velocity(), body.position(), target)) {
                return velocityToBeGained(target);
            }
            return commandFrom(fix, dt);
        }
        return ControlCommand.coastWithThrust(1.0);
    }

    /**
     * Boost-phase alignment by velocity-to-be-gained (RC3).
     *
     * <p>The obvious implementation - point the nose at the target and burn - does not work, and
     * the reason is worth stating because it cost a sweep of failed intercepts to find. Pointing
     * at the target adds velocity <em>toward</em> the target but does nothing about the velocity
     * the missile already has going somewhere else. A weapon launched 90 degrees across the
     * target bearing keeps every metre per second of its original crosswise motion, arrives with
     * a hundred metres of cross-track offset, and PN gets a fraction of a second to fix what the
     * whole boost should have prevented. Every off-boresight shot missed.
     *
     * <p>What works is what real boost-phase guidance does: compute the velocity the missile
     * <em>wants</em> - pointed at the predicted intercept, at the speed the remaining propellant
     * can buy - and thrust along the difference. That vector has a component that cancels the
     * unwanted crosswise motion, so the missile arrives on a collision course rather than merely
     * pointed at one.
     */
    private ControlCommand velocityToBeGained(Target target) {
        double expectedSpeed = Math.max(body.speed() + remainingDeltaV(), 1.0);

        // Solve for the intercept using the speed the missile EXPECTS to make good, not its
        // current closing velocity. That distinction is the whole fix: a weapon launched across
        // the target bearing is barely closing at the moment of launch, so a closing-velocity
        // estimate returns an intercept tens of seconds away and an aim point in the wrong
        // hemisphere entirely. Iterating on expected speed converges in a few passes.
        double flightTime = target.position().sub(body.position()).length() / expectedSpeed;
        for (int i = 0; i < 8; i++) {
            double next = target.predicted(flightTime).position()
                    .sub(body.position()).length() / expectedSpeed;
            if (Math.abs(next - flightTime) < 1e-4) { flightTime = next; break; }
            flightTime = next;
        }

        Vec3 aim = target.predicted(flightTime).position();
        Vec3 toAim = aim.sub(body.position()).normalized();
        if (toAim.lengthSq() == 0.0) {
            return ControlCommand.pointAt(target.position().sub(body.position()), 1.0);
        }
        Vec3 vtg = toAim.scale(expectedSpeed).sub(body.velocity());
        if (vtg.lengthSq() < 1e-12) return ControlCommand.pointAt(toAim, 1.0);
        return ControlCommand.pointAt(vtg, 1.0);
    }

    /** Delta-v the current stage can still deliver, by Tsiolkovsky on its remaining propellant. */
    private double remainingDeltaV() {
        var stage = body.currentStage();
        if (stage == null || body.stageFuel() <= 0.0) return 0.0;
        double mass = body.mass();
        double after = mass - body.stageFuel();
        if (after <= 0.0) return 0.0;
        return stage.exhaustVelocityVacuum(engineFrame) * Math.log(mass / after);
    }

    private ControlCommand guidedCommand(double dt, Target target,
                                         List<Countermeasures.Decoy> decoys, EventSink events) {
        if (seeker == null || target == null) return ControlCommand.coast();
        return commandFrom(updateSeeker(dt, target, decoys, events), dt);
    }

    /**
     * Advance the seeker exactly once per tick and return what it believes.
     *
     * <p>Exactly once matters: calling it twice would run the lock state machine and the
     * memory-track clock at double rate, which is both wrong and a determinism break (I7).
     */
    private Seeker.Fix updateSeeker(double dt, Target target,
                                    List<Countermeasures.Decoy> decoys, EventSink events) {
        Vec3 boresight = body.orientation().forward();
        Seeker.Fix fix = seeker.update(body.position(), body.velocity(), boresight, target,
                env.world(), dt, body.age(), events);

        if (decoys != null && !decoys.isEmpty()) {
            Countermeasures.attemptSeduction(seeker, body.profile().seeker(), body.position(),
                    target, decoys, body.age(), seed, events);
        }
        return fix;
    }

    /** Turn a seeker fix into a steering command. */
    private ControlCommand commandFrom(Seeker.Fix fix, double dt) {
        double throttle = body.phase().isPowered() ? 1.0 : 0.0;
        if (fix == null || !fix.guides() || fix.target() == null) {
            return ControlCommand.coastWithThrust(throttle);
        }
        Vec3 accel = pn.command(body.position(), body.velocity(), fix.target());
        if (loft != null) {
            accel = loft.blend(accel, env.altitudeOf(body.position().y()), body.age(), dt);
        }
        return ControlCommand.accelerate(accel, throttle);
    }

    /** Whether a launch reached the Karman line under power and is awaiting an insertion call. */
    public boolean awaitingInsertion() {
        return insertionAttempted && body.phase() == FlightPhase.MIDCOURSE;
    }

    /** Move a body into the registry after a successful insertion. */
    public void markInserted(EventSink events) {
        body.phases().transition(FlightPhase.ORBIT, body.age(), "orbital insertion", events);
    }

    /** Give up on orbit: the body is ballistic and will come back down. */
    public void markInsertionFailed(EventSink events) {
        if (body.phase() == FlightPhase.MIDCOURSE) {
            body.phases().transition(FlightPhase.DESCENT, body.age(),
                    "insufficient delta-v for insertion", events);
        }
    }

    public KineticBody body() { return body; }

    public Mission mission() { return mission; }
}
