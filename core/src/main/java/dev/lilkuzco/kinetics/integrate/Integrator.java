package dev.lilkuzco.kinetics.integrate;

import dev.lilkuzco.kinetics.aero.Aerodynamics;
import dev.lilkuzco.kinetics.body.KineticBody;
import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.env.Environment;
import dev.lilkuzco.kinetics.event.EventSink;
import dev.lilkuzco.kinetics.event.KineticEvent;
import dev.lilkuzco.kinetics.invariant.Invariants;
import dev.lilkuzco.kinetics.math.Quat;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.profile.Airframe;
import dev.lilkuzco.kinetics.profile.Profile;

/**
 * Semi-implicit (symplectic) Euler with substepping and swept collision (RA1).
 *
 * <p><b>The ordering is the whole point.</b> Velocity is updated from acceleration first, and
 * <em>then</em> position is updated from the new velocity:
 * <pre>
 *     v += a * dt
 *     x += v * dt        // the already-updated v
 * </pre>
 * Doing it the other way round - explicit Euler - makes a ballistic body gain energy every
 * step, so a shell fired on a flat trajectory slowly climbs and an orbit spirals outward.
 * Semi-implicit Euler instead loses a fixed {@code g^2 dt^2 / 2} per step, bounded and
 * always in the same direction, which is why invariant I3 can be stated as "energy may only
 * fall" and actually hold.
 *
 * <p>Guidance never touches the state update. It asks for a lateral acceleration; this class
 * decides how much of that the airframe can produce from the lift available at the current
 * dynamic pressure, and the leftover request is simply not honoured. That is where I6's turn
 * authority comes from, and why a slow body cannot turn like a fast one.
 */
public final class Integrator {

    private final Constants k;
    private final Invariants invariants;
    private final double g0;
    private final double targetDisplacement;
    private final int maxSubsteps;
    private final double suttonGravesK;
    private final double gLimitHardCap;
    private final double substepVelocityFraction;
    private final double minSubstepReferenceSpeed;

    public Integrator(Constants k) { this(k, new Invariants(k)); }

    public Integrator(Constants k, Invariants invariants) {
        this.k = k;
        this.invariants = invariants;
        this.g0 = k.d("gravity.g0");
        this.targetDisplacement = k.d("limits.max_substep_displacement");
        this.maxSubsteps = k.i("limits.max_substeps");
        this.suttonGravesK = k.d("reentry.sutton_graves_k");
        this.gLimitHardCap = k.d("limits.g_limit_hard_cap");
        this.substepVelocityFraction = k.d("limits.max_substep_velocity_fraction");
        this.minSubstepReferenceSpeed = k.d("limits.min_substep_reference_speed");
    }

    /** What one tick of integration did. */
    public record StepResult(int substepsUsed, boolean collided,
                             SweptCollision.Hit hit, double maxDynamicPressure,
                             double peakHeating) {}

    /**
     * Advance one server tick.
     *
     * @param tickDt normally 1/20 s; taken as a parameter so tests can integrate at other rates
     */
    public StepResult step(KineticBody body, Environment env, ControlCommand control,
                           double worldTime, double tickDt, EventSink events) {

        int substeps = planSubsteps(body, env, tickDt);
        double dt = tickDt / substeps;

        Profile profile = body.profile();
        Airframe airframe = profile.airframe();
        Aerodynamics aero = body.aerodynamics();
        double gravity = env.gravity();
        double gLimitAccel = Math.min(airframe.gLimitG(), gLimitHardCap) * g0;

        boolean collided = false;
        SweptCollision.Hit hit = null;
        double maxQ = 0.0;
        double peakHeat = 0.0;

        for (int s = 0; s < substeps && !collided; s++) {
            double subTime = worldTime + s * dt;
            Vec3 pos = body.position();
            Vec3 vel = body.velocity();
            double mass = body.mass();

            double altitude = env.altitudeOf(pos.y());
            double rho = env.densityAt(pos.y());
            double pressureRatio = env.pressureRatioAt(pos.y());
            Vec3 airspeed = env.airspeedOf(pos, vel, subTime);
            double airspeedMag = airspeed.length();
            double mach = env.machAt(airspeedMag, pos.y());
            double q = 0.5 * rho * airspeedMag * airspeedMag;
            if (q > maxQ) maxQ = q;

            double energyBefore = body.specificEnergy(gravity, altitude);
            boolean powered = body.phase().isPowered() && control.wantsThrust() && body.hasFuel();

            // ---- attitude ------------------------------------------------
            Vec3 desired = desiredForward(body, control, airspeed, q, mass, aero);
            slewToward(body, desired, dt, rho, control);
            Vec3 forward = body.orientation().forward();

            // ---- aerodynamic forces (RB2, RB4, RB4b, RB5) -----------------
            Aerodynamics.AeroResult res = aero.compute(airspeed, forward, rho, mach,
                    airframe.referenceArea(), airframe.wingArea());
            Vec3 dragForce = res.dragForce();
            Vec3 liftForce = res.liftForce();

            // A deployed canopy is extra drag area on the same airflow, not a new force
            // direction. Added as C_d*A so a light capsule under a big chute stays correct.
            if (body.hasInflatedChute() && airspeedMag > 1e-9) {
                Vec3 airDir = airspeed.scale(1.0 / airspeedMag);
                dragForce = dragForce.add(airDir.scale(-q * body.chuteCdA()));
            }
            invariants.checkDrag(body, dragForce, airspeed);

            // ---- thrust (RD2, RD2b) --------------------------------------
            double thrustMag = 0.0;
            boolean hadFuel = body.hasFuel();
            if (powered) {
                body.setThrottle(control.throttle());
                thrustMag = body.burn(dt, pressureRatio);
                body.addAchievedDeltaV(thrustMag / mass * dt);
            }
            invariants.checkThrust(body, thrustMag, hadFuel);
            Vec3 thrustForce = forward.scale(thrustMag);

            // ---- I2: clamp the commanded accelerations, not the natural ones
            // Lift and thrust are what the vehicle asks for and are subject to the airframe's
            // g-limit. Gravity and drag are done to it by the universe and are not clamped -
            // clamping drag would let a body survive a dive it should not.
            Vec3 commandedAccel = liftForce.add(thrustForce).scale(1.0 / mass)
                    .clampLength(gLimitAccel);

            // ---- drag, with a reversal guard -----------------------------
            // Drag can bring a body to rest relative to the air; it can never push it
            // backwards. Without this, one stiff substep under a freshly opened canopy would
            // flip the velocity and manufacture energy.
            Vec3 dragAccel = dragForce.scale(1.0 / mass);
            double dragDeltaV = dragAccel.length() * dt;
            if (dragDeltaV > airspeedMag && airspeedMag > 1e-9) {
                dragAccel = dragAccel.scale(airspeedMag / dragDeltaV);
            }

            // ---- RA1: velocity THEN position -----------------------------
            //
            // The commanded acceleration is split about the velocity direction and the two
            // halves are integrated differently, because they mean different things.
            //
            // Adding a PERPENDICULAR acceleration is the classic way to manufacture energy in an
            // Euler integrator: |v + a_perp*dt|^2 = |v|^2 + |a_perp*dt|^2, so a pure turn speeds
            // the body up by half the square of the increment. Usually that is lost in the
            // noise. It is not lost in the noise for a 90 g airframe at 3 m/s, where a*dt is
            // larger than v itself - the fuzz harness found exactly that case and it gained 40
            // J/kg in a single substep. Lift does no work, so it is applied as a ROTATION of the
            // velocity through atan2(a_perp*dt, v): the direction the additive form would have
            // produced, with the magnitude left alone. Exact at any step size.
            //
            // The PARALLEL component, drag and gravity stay additive - those genuinely change
            // speed, and semi-implicit Euler's small energy loss on them is what keeps I3 honest.
            Vec3 passive = dragAccel.add(env.gravityVector());
            Vec3 turned = vel;
            Vec3 alongVelocity = commandedAccel;

            double speedNow = vel.length();
            if (speedNow > 1e-9 && commandedAccel.lengthSq() > 1e-24) {
                alongVelocity = commandedAccel.projectOnto(vel);
                Vec3 perpendicular = commandedAccel.sub(alongVelocity);
                double perpMag = perpendicular.length();
                if (perpMag > 1e-12) {
                    Vec3 axis = vel.cross(perpendicular);
                    if (axis.lengthSq() > 1e-24) {
                        double theta = Math.atan2(perpMag * dt, speedNow);
                        turned = Quat.fromAxisAngle(axis.normalized(), theta).rotate(vel);
                    }
                }
            }

            Vec3 newVel = turned.add(alongVelocity.scale(dt)).add(passive.scale(dt));
            Vec3 displacement = newVel.scale(dt);
            Vec3 newPos = pos.add(displacement);

            // ---- swept collision (I1) ------------------------------------
            SweptCollision.Hit h = SweptCollision.cast(env.world(), pos, newPos);
            if (h != null) {
                collided = true;
                hit = h;
                // Stop just short of the face so the body is not left inside the block it hit.
                body.setPosition(h.point().add(h.normal().scale(1e-4)));
                body.setVelocity(newVel);
                events.accept(new KineticEvent.Impact(body.id(), body.age() + (s + 1) * dt,
                        body.position(), newVel, mass,
                        "block[" + h.blockX() + "," + h.blockY() + "," + h.blockZ() + "]"));
                body.setVelocity(Vec3.ZERO);
            } else {
                body.setVelocity(newVel);
                body.setPosition(newPos);
            }

            // ---- derived state -------------------------------------------
            double heating = heatingRate(rho, airspeedMag, airframe.noseRadius());
            if (heating > peakHeat) peakHeat = heating;
            body.recordFlightState(q, mach, res.angleOfAttackDeg(), res.stalled());
            body.recordHeating(heating);

            // ---- invariants ----------------------------------------------
            invariants.checkContinuity(body, displacement, substeps, rho);
            invariants.checkQuaternion(body);
            if (!powered && !collided) {
                double energyAfter = body.specificEnergy(gravity, env.altitudeOf(body.position().y()));
                // Bound on the work the wind could have done this substep. Zero with wind off.
                //
                // It has to cover LIFT as well as drag. Both are defined relative to the air, so
                // in a wind both acquire a component along the ground velocity and both can do
                // real work in the world frame - lift on a body being blown sideways is no
                // different from drag on one being blown along.
                double windSpeed = env.wind().isEnabled()
                        ? vel.sub(airspeed).length() : 0.0;
                double windAllowance = windSpeed <= 0.0 ? 0.0
                        : (dragForce.length() + liftForce.length()) * windSpeed * dt / mass;
                invariants.checkEnergy(body, energyBefore, energyAfter, gravity, altitude,
                        windAllowance);
            }
        }

        body.advanceAge(tickDt);
        return new StepResult(substeps, collided, hit, maxQ, peakHeat);
    }

    /**
     * How many substeps this tick needs (RA1).
     *
     * <p>The floor is the profile's declared count; the planner raises it until the predicted
     * displacement per substep meets the target, and the cap bounds the cost. When the cap
     * binds - a body at orbital speed low in the atmosphere - the step is coarser than the
     * target but still well inside the hard I1 bound, and collision remains exact regardless
     * because traversal is swept.
     */
    private int planSubsteps(KineticBody body, Environment env, double tickDt) {
        int floor = body.profile().effectiveSubsteps(k);
        double speed = body.speed();

        // Rule one: displacement, so the air density driving drag does not go stale.
        int needed = (int) Math.ceil(speed * tickDt / targetDisplacement);

        // Rule two: velocity change, which is what the displacement rule misses entirely. A
        // 0.08 kg body with 25 m^2 of drag area pulls 350 m/s^2 while ambling along at 2 m/s -
        // it covers nothing, so rule one is satisfied at four substeps, and meanwhile its
        // velocity vector swings through 70 degrees per substep. Nothing first-order is
        // trustworthy there, and the fuzz harness proved it by manufacturing energy.
        double accel = estimatePeakAcceleration(body, env, speed);
        double reference = Math.max(speed, minSubstepReferenceSpeed);
        int forAccel = (int) Math.ceil(accel * tickDt / (substepVelocityFraction * reference));

        return Math.max(1, Math.min(maxSubsteps, Math.max(floor, Math.max(needed, forAccel))));
    }

    /**
     * Cheap upper bound on the acceleration this body is about to feel, for substep planning.
     * Deliberately an overestimate: planning too finely costs a little time, planning too
     * coarsely costs correctness.
     */
    private double estimatePeakAcceleration(KineticBody body, Environment env, double speed) {
        Airframe airframe = body.profile().airframe();
        double mass = body.mass();
        if (mass <= 0.0) return 0.0;

        double rho = env.densityAt(body.position().y());
        double q = 0.5 * rho * speed * speed;
        double dragArea = airframe.cd0() * airframe.referenceArea() + body.chuteCdA();
        double aDrag = q * dragArea / mass;

        double clMax = body.aerodynamics().liftCurve().clMax();
        double aLift = Math.min(q * clMax * airframe.wingArea() / mass,
                Math.min(airframe.gLimitG(), gLimitHardCap) * g0);

        double aThrust = 0.0;
        var stage = body.currentStage();
        if (stage != null && body.hasFuel() && body.phase().isPowered()) {
            aThrust = stage.thrustVacuum() / mass;
        }
        return aDrag + aLift + aThrust + env.gravity();
    }

    /**
     * Where the nose should point this substep.
     *
     * <p>For a lateral-acceleration command this is the inverse of the lift equation: work out
     * the C_L needed to make the requested acceleration at this dynamic pressure, refuse to
     * exceed C_L,max, and convert what is left into an angle of attack. Two limits therefore
     * bind automatically and in the right regimes - a slow body runs out of lift (stall) and a
     * fast one runs into the g-limit downstream - which is exactly I6.
     */
    private Vec3 desiredForward(KineticBody body, ControlCommand control, Vec3 airspeed,
                                double q, double mass, Aerodynamics aero) {
        Vec3 airDir = airspeed.normalized();
        Vec3 currentForward = body.orientation().forward();

        return switch (control.mode()) {
            case COAST ->
                // Weathervane into the airflow. With no air there is nothing to weathervane
                // against, so attitude is held (or spins freely on its angular velocity).
                    airDir.lengthSq() > 0.0 ? airDir : currentForward;
            case DIRECTION -> control.target().lengthSq() > 0.0 ? control.target() : currentForward;
            case LATERAL_ACCEL -> {
                double wingArea = body.profile().airframe().wingArea();
                Vec3 lateral = control.target().perpendicularTo(airDir);
                if (wingArea <= 0.0 || q <= 0.0 || lateral.lengthSq() < 1e-18
                        || airDir.lengthSq() == 0.0) {
                    // No lifting surface, or no air to work against: this body cannot steer
                    // aerodynamically at all, and pretending otherwise is exactly the kind of
                    // free manoeuvring I6 exists to prevent.
                    yield airDir.lengthSq() > 0.0 ? airDir : currentForward;
                }
                double clNeeded = lateral.length() * mass / (q * wingArea);
                double cl = Math.min(clNeeded, aero.liftCurve().clMax());
                double slope = aero.liftCurve().clMax() / aero.liftCurve().stallAoaDeg();
                double aoaRad = Math.toRadians(slope > 0.0 ? cl / slope : 0.0);
                yield airDir.scale(Math.cos(aoaRad))
                        .add(lateral.normalized().scale(Math.sin(aoaRad)))
                        .normalized();
            }
        };
    }

    /**
     * Rotate the nose toward {@code desired}, no faster than the profile's slew rate.
     *
     * <p>This is where I6's "instant reversals impossible by construction" lives. There is no
     * code path that sets an orientation directly from a guidance command - every attitude
     * change goes through a bounded rotation, so a seeker demanding a 180-degree flip gets a
     * turn that takes as long as the airframe needs.
     */
    private void slewToward(KineticBody body, Vec3 desired, double dt, double rho,
                            ControlCommand control) {
        Vec3 current = body.orientation().forward();
        if (desired.lengthSq() < 1e-18) return;

        if (control.mode() == ControlCommand.Mode.COAST && rho <= 0.0) {
            // Vacuum coast: no aerodynamic moment exists to turn the body, so it keeps
            // whatever rotation it already had. This is what makes spin stabilisation work
            // (RE6) and why an unpowered body in vacuum cannot reorient itself for free.
            body.setOrientation(body.orientation().integrate(body.angularVelocity(), dt));
            return;
        }

        double angle = current.angleTo(desired);
        if (angle < 1e-9) {
            body.setAngularVelocity(Vec3.ZERO);
            return;
        }
        double maxStep = Math.toRadians(body.profile().maxSlewRateDeg()) * dt;
        double step = Math.min(angle, maxStep);

        Vec3 axis = current.cross(desired);
        if (axis.lengthSq() < 1e-18) {
            // Exactly antiparallel: pick a deterministic perpendicular so I7 holds.
            axis = Math.abs(current.x()) < 0.9 ? new Vec3(1, 0, 0) : new Vec3(0, 1, 0);
            axis = current.cross(axis);
        }
        axis = axis.normalized();

        Quat rotation = Quat.fromAxisAngle(axis, step);
        body.setOrientation(rotation.multiply(body.orientation()));
        body.setAngularVelocity(axis.scale(step / dt));
    }

    /**
     * Stagnation-point heating rate, W/m^2 (RE7). Sutton-Graves in its simplified form:
     * {@code q = K * sqrt(rho / R_n) * v^3}.
     *
     * <p>The cube on velocity and the square root on nose radius together give the blunt-body
     * result: a wide, draggy shape sheds its speed high up where rho is small, and because
     * heating goes as {@code v^3} that early deceleration cuts peak heating far more than the
     * extra drag costs. A slender body keeps its speed until it is deep in thick air and cooks.
     * The test battery asserts this ordering directly rather than assuming it.
     */
    private double heatingRate(double rho, double speed, double noseRadius) {
        if (rho <= 0.0 || speed <= 0.0 || noseRadius <= 0.0) return 0.0;
        return suttonGravesK * Math.sqrt(rho / noseRadius) * speed * speed * speed;
    }

    public Invariants invariants() { return invariants; }

    public Constants constants() { return k; }
}
