package dev.lilkuzco.kinetics.test;

import dev.lilkuzco.kinetics.body.KineticBody;
import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.env.Environment;
import dev.lilkuzco.kinetics.env.WorldProbe;
import dev.lilkuzco.kinetics.event.EventSink;
import dev.lilkuzco.kinetics.event.KineticEvent;
import dev.lilkuzco.kinetics.guidance.Seeker;
import dev.lilkuzco.kinetics.guidance.Target;
import dev.lilkuzco.kinetics.integrate.Integrator;
import dev.lilkuzco.kinetics.math.Quat;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.orbit.Attitude;
import dev.lilkuzco.kinetics.orbit.Orbit;
import dev.lilkuzco.kinetics.orbit.OrbitalRegistry;
import dev.lilkuzco.kinetics.phase.FlightDirector;
import dev.lilkuzco.kinetics.phase.FlightPhase;
import dev.lilkuzco.kinetics.profile.EngineFrame;
import dev.lilkuzco.kinetics.profile.Profile;
import dev.lilkuzco.kinetics.propulsion.Propulsion;
import dev.lilkuzco.kinetics.sensors.Countermeasures;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Golden trajectories: fourteen full scenarios, each asserted on behaviour <em>and</em> hashed.
 *
 * <p>The two halves do different jobs. The behavioural assertions say the scenario did the right
 * thing - the missile hit, the chute shredded, the orbit decayed. The hash says the scenario did
 * <em>exactly</em> what it did last time, down to the last bit of every position (I7). A change
 * to the physics that improves accuracy will break the hashes, and that is correct: the hashes
 * are re-recorded deliberately, never adjusted quietly.
 *
 * <p>Run with {@code -Dkinetics.golden.record=<path>} to write the reference file.
 */
public final class GoldenTests {

    private static final String RESOURCE = "/golden-trajectories.txt";

    private GoldenTests() {}

    private final Map<String, String> recorded = new LinkedHashMap<>();

    public static void run(Harness h, Constants k) {
        Map<String, String> expected = loadExpected();
        Map<String, String> actual = new LinkedHashMap<>();

        h.suite("Golden trajectories — behaviour");
        mortarArc(h, k, actual);
        pnInterceptOfManeuveringTarget(h, k, actual);
        loftedFullStateMachine(h, k, actual);
        offBoresightLaunch(h, k, actual);
        terrainMaskedLockCycle(h, k, actual);
        pointDefenceIntercept(h, k, actual);
        decoySeduction(h, k, actual);
        twoStageToOrbit(h, k, actual);
        liftoffFailure(h, k, actual);
        failedInsertion(h, k, actual);
        insertionAndPasses(h, k, actual);
        decayingLowOrbit(h, k, actual);
        overQChuteShred(h, k, actual);
        drogueThenMain(h, k, actual);
        deorbitToLanding(h, k, actual);
        h.endSuite();

        h.suite("Golden trajectories — committed hashes (I7)");
        String record = System.getProperty("kinetics.golden.record");
        if (record != null) {
            writeRecord(Path.of(record), actual);
            h.note("RECORDED " + actual.size() + " hashes to " + record
                    + " — reference file rewritten, not compared.");
            for (var e : actual.entrySet()) h.metric(e.getKey(), e.getValue().substring(0, 16));
        } else if (expected.isEmpty()) {
            h.check("reference hashes present", false,
                    "no " + RESOURCE + " on the classpath — run the build with 'golden-record'");
        } else {
            for (var e : actual.entrySet()) {
                String want = expected.get(e.getKey());
                if (want == null) {
                    h.check(e.getKey(), false, "no committed hash for this scenario");
                } else {
                    h.equalStrings(e.getKey(), e.getValue(), want);
                }
            }
            for (String key : expected.keySet()) {
                if (!actual.containsKey(key)) {
                    h.check(key, false, "committed hash exists but the scenario did not run");
                }
            }
        }
        h.endSuite();
    }

    // ---- 1. mortar arc ----------------------------------------------------

    private static void mortarArc(Harness h, Constants k, Map<String, String> out) {
        Profile p = Sim.profile(k, "kinetics:mortar_shell");
        Environment env = ground(k);
        KineticBody body = Sim.body("mortar", p, k, origin(k),
                launch(new Vec3(1, 0, 0), 150.0, 55.0), FlightPhase.DESCENT);
        FlightDirector director = new FlightDirector(k, env, body,
                FlightDirector.Mission.BALLISTIC, new Integrator(k), 1L);
        Sim.Trace trace = Sim.fly(body, env, director, k, 20000, 1, null, null);

        var impact = trace.events.first(KineticEvent.Impact.class);
        h.isTrue("mortar arc: lands", impact != null && trace.landed,
                impact == null ? "no impact" : String.format(
                        "range %.2f m, apex %.2f m, flight %.2f s, impact %.1f m/s",
                        Math.hypot(impact.position().x(), impact.position().z()),
                        trace.peakAltitude, body.age(), impact.velocity().length()));
        out.put("mortar_arc", trace.hash());
    }

    // ---- 2. PN intercept of a manoeuvring target --------------------------

    private static void pnInterceptOfManeuveringTarget(Harness h, Constants k,
                                                       Map<String, String> out) {
        Engagement e = engage(k, "kinetics:interceptor",
                new Vec3(0, Sim.y(k, 60.0), 0), new Vec3(40, 0, 0),
                new Vec3(320, Sim.y(k, 90.0), 40), new Vec3(-22, 0, 6),
                new Vec3(0, 0, 9.0), 600, 1L, WorldProbe.empty(), List.of());

        h.isTrue("PN intercept of a manoeuvring target: fuse fired",
                e.proximity != null,
                e.proximity == null
                        ? String.format("MISS — closest approach %.3f m after %.2f s",
                                e.director.closestApproach(), e.body.age())
                        : String.format("miss distance %.3f m at t=%.2f s, closing %.1f m/s",
                                e.proximity.missDistance(), e.proximity.bodyAge(),
                                e.proximity.velocity().length()));
        h.isTrue("  it arrived while still under power",
                e.reachedPhases.contains(FlightPhase.BOOST)
                        && e.reachedPhases.contains(FlightPhase.TERMINAL)
                        && !e.reachedPhases.contains(FlightPhase.MIDCOURSE),
                e.reachedPhases + " — a 1.25 s motor and a 1.3 s intercept, so it never coasts. "
                        + "The lofted case below exercises MIDCOURSE.");
        out.put("pn_intercept_maneuvering", e.trace.hash());
    }

    // ---- 2b. lofted, full state machine (RC4) ------------------------------

    private static void loftedFullStateMachine(Harness h, Constants k, Map<String, String> out) {
        Profile p = Sim.profile(k, "kinetics:lofted_missile");
        Environment env = Environment.overworld(k, WorldProbe.empty());
        Engagement e = new Engagement();
        e.body = Sim.body("lofted", p, k, new Vec3(0, Sim.y(k, 20.0), 0), new Vec3(35, 3, 0),
                FlightPhase.RAIL);
        e.director = new FlightDirector(k, env, e.body, FlightDirector.Mission.GUIDED,
                new Integrator(k), 12L)
                .withLoft(new dev.lilkuzco.kinetics.guidance.LoftProfile(k, 10.0, 55.0, 0.5));
        e.trace = new Sim.Trace();

        double dt = k.d("world.tick_seconds");
        Vec3 tp0 = new Vec3(900, Sim.y(k, 40.0), 0);
        Vec3 tv = new Vec3(-14, 0, 5);

        for (int tick = 0; tick < 1600 && e.body.phase().isInWorld(); tick++) {
            if (!e.reachedPhases.contains(e.body.phase())) e.reachedPhases.add(e.body.phase());
            double t = tick * dt;
            Target target = new Target("bogey", tp0.add(tv.scale(t)), tv, Vec3.ZERO, 5.0, false);
            e.director.tick(t, dt, target, List.of(), e.trace.events);
            e.trace.states.add(e.body.snapshot());
        }
        if (!e.reachedPhases.contains(e.body.phase())) e.reachedPhases.add(e.body.phase());
        e.proximity = e.trace.events.first(KineticEvent.Proximity.class);

        h.isTrue("lofted missile: flew the whole state machine",
                e.reachedPhases.containsAll(List.of(FlightPhase.RAIL, FlightPhase.BOOST,
                        FlightPhase.MIDCOURSE, FlightPhase.TERMINAL)),
                e.reachedPhases.toString());
        double apex = e.trace.states.stream()
                .mapToDouble(s -> env.altitudeOf(s.position().y())).max().orElse(0.0);
        h.isTrue("  it lofted, then came down on the target",
                apex > 60.0,
                String.format("apex %.1f m against a target at 40 m", apex));
        h.isTrue("  and hit",
                e.proximity != null,
                e.proximity == null
                        ? String.format("MISS — closest %.3f m", e.director.closestApproach())
                        : String.format("miss distance %.3f m at t=%.2f s over %.0f m of range",
                                e.proximity.missDistance(), e.proximity.bodyAge(), 900.0));
        out.put("lofted_full_state_machine", e.trace.hash());
    }

    // ---- 3. off-boresight alignment pre-phase (RC3) ------------------------

    private static void offBoresightLaunch(Harness h, Constants k, Map<String, String> out) {
        // Launched 120 degrees across the target bearing - nearly backwards, and far outside the
        // small-angle geometry PN was derived for. The turn is in the HORIZONTAL plane: a
        // near-180-degree reversal in the VERTICAL plane is a different problem entirely,
        // because the nose sweeps through vertical while the motor is lit and the missile throws
        // itself into a climb, ending up above its own seeker cone. That is a real failure mode,
        // not a modelling artefact, and it is why vertical-launch weapons turn over before the
        // main motor lights.
        double offBoresightDeg = 120.0;
        double r = Math.toRadians(offBoresightDeg);
        Vec3 launchPos = new Vec3(0, Sim.y(k, 80.0), 0);
        Vec3 launchVel = new Vec3(Math.cos(r) * 40.0, 0, -Math.sin(r) * 40.0);
        Vec3 targetPos = new Vec3(800, Sim.y(k, 90.0), 0);

        Engagement e = engage(k, "kinetics:lofted_missile", launchPos, launchVel,
                targetPos, new Vec3(-15, 0, 0), Vec3.ZERO, 1200, 2L,
                WorldProbe.empty(), List.of());

        Profile p = Sim.profile(k, "kinetics:lofted_missile");
        var pn = new dev.lilkuzco.kinetics.guidance.ProportionalNavigation(k, p.seeker());
        boolean neededAlignment = pn.needsBoresightAlignment(launchVel, launchPos,
                Target.stationary("t", targetPos));

        h.isTrue("off-boresight launch: alignment pre-phase was required",
                neededAlignment,
                String.format("%.0f deg off boresight is outside PN's derivation, so BOOST flies "
                        + "velocity-to-be-gained first", offBoresightDeg));
        h.isTrue("  and the intercept completed",
                e.proximity != null,
                e.proximity == null
                        ? String.format("MISS — closest %.3f m", e.director.closestApproach())
                        : String.format("miss distance %.3f m after a %.0f deg reversal, "
                                + "t=%.2f s", e.proximity.missDistance(), offBoresightDeg,
                                e.proximity.bodyAge()));

        // RF4 agrees the shot was worth taking: the predicted intercept lands inside the
        // envelope and inside the flight-time budget.
        var envelope = new dev.lilkuzco.kinetics.sensors.EngagementEnvelope(p.seeker());
        var solution = envelope.evaluate(launchPos, launchVel, 250.0,
                Target.moving("t", targetPos, new Vec3(-15, 0, 0)));
        h.isTrue("  RF4 rated the shot acceptable before launch",
                solution.acceptable(), solution.reason());
        out.put("off_boresight_alignment", e.trace.hash());
    }

    // ---- 4. terrain masking: lock loss -> memory track -> reacquire (RC6) ---

    private static void terrainMaskedLockCycle(Harness h, Constants k, Map<String, String> out) {
        // A ridge between launcher and target, and a target that dives behind it and climbs back
        // out. This is the real tactic: the target is visible over the ridge, drops into its
        // shadow, and re-emerges. The seeker must lose lock, coast on its last prediction, and
        // reacquire - not quietly keep tracking through rock.
        int seaLevel = (int) k.d("world.sea_level_y");
        int ridgeTop = seaLevel + 120;
        WorldProbe masked = (x, y, z) ->
                y <= seaLevel - 1 || (x >= 150 && x <= 152 && y <= ridgeTop);

        double dt = k.d("world.tick_seconds");
        Vec3 launchPos = new Vec3(0, Sim.y(k, 100.0), 0);
        Engagement e = engage(k, "kinetics:interceptor", launchPos, new Vec3(45, 0, 0),
                900, 3L, masked, List.of(), tick -> {
                    double t = tick * dt;
                    // Visible over the ridge, then dives into its shadow. The seeker regains it
                    // when the missile itself passes the ridge, which is what makes this a
                    // reacquisition rather than a lucky sample.
                    double altitude = t < 0.2 ? 200.0
                            : t < 0.45 ? 200.0 - (t - 0.2) * 720.0
                            : 20.0;
                    Vec3 p = new Vec3(360.0 - 6.0 * t, Sim.y(k, altitude), 0);
                    return new Target("bogey", p, new Vec3(-6, 0, 0), Vec3.ZERO, 5.0, false);
                });

        var lost = e.trace.events.ofType(KineticEvent.LockLost.class);
        var acquired = e.trace.events.ofType(KineticEvent.LockAcquired.class);
        boolean reacquired = acquired.stream().anyMatch(KineticEvent.LockAcquired::reacquisition);

        h.isTrue("terrain masking: lock was lost behind the ridge",
                !lost.isEmpty(),
                lost.isEmpty() ? "never lost lock" : "cause: " + lost.get(0).cause()
                        + ", memory-track " + lost.get(0).memoryTrackSeconds() + " s");
        h.isTrue("  and reacquired on the far side",
                reacquired,
                acquired.size() + " acquisitions, "
                        + acquired.stream().filter(KineticEvent.LockAcquired::reacquisition).count()
                        + " of them reacquisitions");
        h.isTrue("  the seeker never silently regained truth while masked",
                e.trace.events.ofType(KineticEvent.LockExpired.class).size()
                        <= lost.size(),
                "memory-track expiries never exceed lock losses");
        out.put("terrain_masked_lock_cycle", e.trace.hash());
    }

    // ---- 5. point defence: missile vs missile ------------------------------

    private static void pointDefenceIntercept(Harness h, Constants k, Map<String, String> out) {
        // The target is itself a fast, small, inbound missile. The interceptor is launched on a
        // lead bearing rather than straight up: a point-defence round that has to turn 90
        // degrees first has already lost, which is why real systems slew the launcher.
        Engagement e = engage(k, "kinetics:interceptor",
                new Vec3(0, Sim.y(k, 40.0), 0), new Vec3(45, 30, 0),
                new Vec3(300, Sim.y(k, 160.0), 0), new Vec3(-160, -35, 0),
                Vec3.ZERO, 600, 4L, WorldProbe.empty(), List.of());

        h.isTrue("point defence: inbound missile intercepted",
                e.proximity != null,
                e.proximity == null
                        ? String.format("MISS — closest %.3f m", e.director.closestApproach())
                        : String.format("miss distance %.3f m at t=%.2f s against a %.0f m/s "
                                + "closing target", e.proximity.missDistance(),
                                e.proximity.bodyAge(), 164.0));
        out.put("point_defence_intercept", e.trace.hash());
    }

    // ---- 6. decoy seduction (RF5) -----------------------------------------

    private static void decoySeduction(Harness h, Constants k, Map<String, String> out) {
        // A cheap head against a bright flare: the decoy presents a legitimate signature and
        // outscores the aircraft, and the seeker follows it believing it is right.
        Vec3 targetPos = new Vec3(220, Sim.y(k, 95.0), 0);
        List<Countermeasures.Decoy> decoys = List.of(
                Countermeasures.flare("flare1", targetPos.add(new Vec3(-6, -4, 0)),
                        new Vec3(-10, -8, 0), 40.0, 0.0, 30.0));

        Engagement e = engage(k, "kinetics:cheap_missile",
                new Vec3(0, Sim.y(k, 80.0), 0), new Vec3(60, 4, 0),
                targetPos, new Vec3(-18, 0, 0), Vec3.ZERO, 900, 5L,
                WorldProbe.empty(), decoys);

        var seduced = e.trace.events.ofType(KineticEvent.DecoySeduced.class);
        h.isTrue("decoy seduction: a cheap seeker was pulled onto the flare",
                !seduced.isEmpty(),
                seduced.isEmpty() ? "the head was not seduced"
                        : String.format("%s -> %s via %s at t=%.2f s", seduced.get(0).fromTarget(),
                                seduced.get(0).toDecoy(), seduced.get(0).countermeasure(),
                                seduced.get(0).bodyAge()));
        out.put("decoy_seduction", e.trace.hash());
    }

    // ---- 7. two-stage rocket to orbit -------------------------------------

    private static void twoStageToOrbit(Harness h, Constants k, Map<String, String> out) {
        Launch launch = fly(k, "kinetics:orbital_rocket_2stage", 6L);
        Propulsion propulsion = new Propulsion(k);
        var assessment = propulsion.assess(launch.profile, k.d("gravity.g0"));

        h.isTrue("two-stage to orbit: cleared both gates",
                assessment.reachesOrbit(), assessment.verdict());
        h.isTrue("  gravity turn rotated the velocity vector toward horizontal",
                launch.finalHorizontalFraction > 0.5,
                String.format("%.1f%% of final velocity is horizontal (pitch %.1f deg above "
                        + "horizontal)", launch.finalHorizontalFraction * 100.0,
                        launch.finalPitchDeg));
        h.isTrue("  two staging events fired",
                launch.trace.events.ofType(KineticEvent.Staging.class).size() == 2,
                launch.trace.events.ofType(KineticEvent.Staging.class).size() + " staging events");

        // The regression guard: a launch vehicle must not fly into the ground before burnout.
        h.isTrue("  it never touched the ground during the burn",
                !launch.trace.events.has(KineticEvent.Impact.class),
                launch.trace.events.has(KineticEvent.Impact.class)
                        ? "IMPACT during ascent — the pitch program is flying it into the terrain"
                        : String.format("apex %.0f m during a %.1f s burn",
                                launch.trace.peakAltitude, launch.body.age()));

        OrbitalRegistry registry = new OrbitalRegistry(k);
        var result = registry.attemptInsertion("sat-2stage", launch.body.achievedDeltaV(),
                0.0, 45.0, 0.0, 0.0, launch.trace.events);
        h.isTrue("  insertion succeeded", result.inserted(),
                String.format("%.1f m/s achieved vs %.1f m/s budget — %s",
                        result.achievedDeltaV(), result.requiredDeltaV(), result.detail()));
        if (result.inserted()) {
            h.metric("  resulting orbit",
                    String.format("altitude %.0f m, period %.1f s",
                            result.altitude(),
                            registry.mechanics().period(result.orbit().semiMajorAxisAtEpoch())));
        }
        out.put("two_stage_to_orbit", launch.trace.hash());
    }

    // ---- 8. T/W < 1 liftoff failure ---------------------------------------

    private static void liftoffFailure(Harness h, Constants k, Map<String, String> out) {
        Launch launch = fly(k, "kinetics:orbital_rocket_twr_fail", 7L);
        var failure = launch.trace.events.first(KineticEvent.LiftoffFailure.class);

        h.isTrue("T/W < 1: the vehicle never left the pad",
                failure != null && launch.body.phase() == FlightPhase.TERMINATED,
                failure == null ? "no liftoff-failure event, phase " + launch.body.phase()
                        : String.format("T/W %.4f against a required %.2f — refused at ignition",
                                failure.twr(), failure.required()));
        h.isTrue("  and it would have looked fine on vacuum thrust",
                vacuumTwr(k, launch.profile) > 1.0,
                String.format("sea-level T/W %.3f, vacuum T/W %.3f — RD5 specifies sea level "
                        + "for exactly this reason",
                        launch.profile.liftoffTwr(k.d("gravity.g0"), EngineFrame.of(k)),
                        vacuumTwr(k, launch.profile)));
        out.put("twr_liftoff_failure", launch.trace.hash());
    }

    private static double vacuumTwr(Constants k, Profile p) {
        EngineFrame frame = EngineFrame.of(k);
        return p.stages().get(0).effectiveThrust(0.0, frame)
                / (p.wetMass() * k.d("gravity.g0"));
    }

    // ---- 9. failed insertion -> ballistic fall ----------------------------

    private static void failedInsertion(Harness h, Constants k, Map<String, String> out) {
        Launch launch = fly(k, "kinetics:orbital_rocket_underpowered", 8L);
        OrbitalRegistry registry = new OrbitalRegistry(k);
        EventSink.Recording events = new EventSink.Recording();
        var result = registry.attemptInsertion("sat-fail", launch.body.achievedDeltaV(),
                0.0, 45.0, 0.0, 0.0, events);

        h.isTrue("underpowered launch: insertion refused",
                !result.inserted() && events.has(KineticEvent.InsertionFailed.class),
                result.detail());
        h.isTrue("  the registry is empty",
                registry.size() == 0, registry.size() + " satellites registered");
        h.isTrue("  and it did lift off cleanly, so this is a delta-v failure not a thrust one",
                !launch.trace.events.has(KineticEvent.LiftoffFailure.class),
                String.format("sea-level T/W %.3f",
                        launch.profile.liftoffTwr(k.d("gravity.g0"), EngineFrame.of(k))));
        out.put("failed_insertion", launch.trace.hash());
    }

    // ---- 10. insertion + three predicted passes ---------------------------

    private static void insertionAndPasses(Harness h, Constants k, Map<String, String> out) {
        OrbitalRegistry registry = new OrbitalRegistry(k);
        var mechanics = registry.mechanics();
        double altitude = k.d("orbit.reference_orbit_altitude");
        Orbit orbit = Orbit.circular("recon", mechanics, 0.0, altitude, 51.6, 0.0, 0.0);
        registry.register(orbit, Attitude.threeAxis(Quat.IDENTITY, 5.0));

        // A ground station directly under the satellite at t=0, so passes certainly exist.
        var start = registry.stateAt("recon", 0.0);
        double stationX = start.groundTrack().worldX();
        double stationZ = start.groundTrack().worldZ();

        List<OrbitalRegistry.Pass> passes = registry.predictPasses("recon", 0.0,
                stationX, stationZ, 30.0, 3, mechanics.period(orbit.semiMajorAxisAtEpoch()) * 4.0);

        h.isTrue("pass prediction: three passes found",
                passes.size() >= 3, passes.size() + " passes within four orbits");
        for (int i = 0; i < Math.min(3, passes.size()); i++) {
            var pass = passes.get(i);
            h.metric(String.format("  pass %d", i + 1),
                    String.format("t=%.1f s, duration %.2f s, closest %.0f m, elevation %.1f deg",
                            pass.entryTime(), pass.durationSeconds(),
                            pass.closestGroundDistance(), pass.maxElevationDeg()));
        }
        if (passes.size() >= 3) {
            double period = mechanics.period(orbit.semiMajorAxisAtEpoch());
            double gap = passes.get(2).entryTime() - passes.get(1).entryTime();
            // Twice per orbit, not once - and that is the rotating frame doing its work. The
            // station sits under the ascending node. Half an orbit later the satellite is at the
            // descending node, 180 degrees away in inertial longitude; but this orbit's period
            // equals one day, so the planet has also turned 180 degrees, and the descending pass
            // lands back over the same ground. Predicted in a non-rotating frame this second
            // pass simply would not exist.
            h.near("  passes recur every half period (ascending and descending nodes)",
                    gap, period * 0.5, 0.05, "s");
            h.metric("  why twice per orbit",
                    String.format("period %.1f s = one day, so the descending node lands back "
                            + "over the station %.1f s later", period, period * 0.5));
        }

        // Hash the ground track itself rather than a flight.
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i <= 240; i++) {
            var s = registry.stateAt("recon", i * 5.0);
            sb.append(Double.doubleToRawLongBits(s.groundTrack().latitudeDeg())).append(':')
              .append(Double.doubleToRawLongBits(s.groundTrack().longitudeDeg())).append(';');
        }
        out.put("insertion_ground_track", sha256(sb.toString()));
    }

    // ---- 11. decaying low orbit -------------------------------------------

    private static void decayingLowOrbit(Harness h, Constants k, Map<String, String> out) {
        OrbitalRegistry registry = new OrbitalRegistry(k);
        var mechanics = registry.mechanics();
        double lowAltitude = k.d("orbit.minimum_sustainable_altitude") * 0.6;
        Orbit orbit = Orbit.circular("doomed", mechanics, 0.0, lowAltitude, 30.0, 0.0, 0.0);
        registry.register(orbit, Attitude.spinStabilized(Quat.IDENTITY, Vec3.UP, 6.0));

        double a0 = registry.stateAt("doomed", 0.0).altitude();
        double a1 = registry.stateAt("doomed", 1200.0).altitude();
        double a2 = registry.stateAt("doomed", 12000.0).altitude();

        h.isTrue("low orbit decays", a1 < a0 && a2 < a1,
                String.format("%.1f m -> %.1f m after one day -> %.1f m after ten",
                        a0, a1, a2));
        h.isTrue("  decay rate grows as it falls",
                registry.decayPerOrbit(a2) > registry.decayPerOrbit(a0),
                String.format("%.2f m/orbit at %.0f m, %.2f m/orbit at %.0f m",
                        registry.decayPerOrbit(a0), a0, registry.decayPerOrbit(a2), a2));

        EventSink.Recording events = new EventSink.Recording();
        var handoffs = registry.advanceDecay(400000.0, events);
        h.isTrue("  and it eventually deorbits on its own",
                !handoffs.isEmpty() && events.has(KineticEvent.Deorbit.class),
                handoffs.isEmpty() ? "still in orbit after 400,000 s"
                        : String.format("deorbited, uncommanded, entering at %.0f m",
                                handoffs.get(0).altitude()));

        // A reference-altitude orbit must NOT decay, or the floor means nothing.
        Orbit stable = Orbit.circular("stable", mechanics, 0.0,
                k.d("orbit.reference_orbit_altitude"), 30.0, 0.0, 0.0);
        registry.register(stable, Attitude.threeAxis(Quat.IDENTITY, 5.0));
        h.near("the reference orbit does not decay",
                registry.stateAt("stable", 1_000_000.0).altitude(),
                k.d("orbit.reference_orbit_altitude"), 1e-12, "m");

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i <= 200; i++) {
            sb.append(Double.doubleToRawLongBits(
                    registry.semiMajorAxisAt(orbit, i * 600.0))).append(';');
        }
        out.put("decaying_low_orbit", sha256(sb.toString()));
    }

    // ---- 12/13. chute deploy gate (RB6) -----------------------------------

    private static void overQChuteShred(Harness h, Constants k, Map<String, String> out) {
        Sim.Trace trace = reentry(k, "kinetics:chute_only_capsule", 1500.0, 9L);
        var shred = trace.events.first(KineticEvent.ChuteShred.class);
        h.isTrue("over-q deploy: the single main chute shredded",
                shred != null,
                shred == null ? "no shred event" : String.format(
                        "'%s' opened at %.0f Pa against a %.0f Pa limit",
                        shred.chuteName(), shred.dynamicPressure(), shred.qDeployMax()));
        h.isTrue("  so nothing slowed it and it arrived fast",
                trace.events.ofType(KineticEvent.ChuteDeployed.class).isEmpty(),
                "no canopy ever inflated");
        out.put("over_q_chute_shred", trace.hash());
    }

    private static void drogueThenMain(Harness h, Constants k, Map<String, String> out) {
        Sim.Trace trace = reentry(k, "kinetics:reentry_capsule", 1500.0, 10L);
        var deployed = trace.events.ofType(KineticEvent.ChuteDeployed.class);
        var impact = trace.events.first(KineticEvent.Impact.class);

        h.isTrue("drogue then main: both canopies opened",
                deployed.size() == 2,
                deployed.stream().map(d -> String.format("%s at %.0f m / %.0f Pa",
                        d.chuteName(), d.altitude(), d.dynamicPressure())).toList().toString());
        h.isTrue("  no canopy shredded",
                !trace.events.has(KineticEvent.ChuteShred.class), "deploy gates respected");

        double landingSpeed = impact == null ? Double.NaN : impact.velocity().length();
        h.isTrue("  and it landed at parachute terminal velocity",
                impact != null && landingSpeed < 15.0,
                String.format("touchdown at %.2f m/s", landingSpeed));
        out.put("drogue_then_main", trace.hash());
    }

    // ---- 14. deorbit -> reentry -> chute -> landing ------------------------

    private static void deorbitToLanding(Harness h, Constants k, Map<String, String> out) {
        OrbitalRegistry registry = new OrbitalRegistry(k);
        var mechanics = registry.mechanics();
        Orbit orbit = Orbit.circular("capsule", mechanics, 0.0,
                k.d("orbit.reference_orbit_altitude"), 40.0, 0.0, 0.0);
        registry.register(orbit, Attitude.threeAxis(Quat.IDENTITY, 5.0));

        EventSink.Recording events = new EventSink.Recording();
        var handoff = registry.deorbit("capsule", 300.0, true, events);
        h.isTrue("commanded deorbit: handed back to the world",
                handoff != null && events.has(KineticEvent.Deorbit.class),
                handoff == null ? "no handoff" : String.format(
                        "entering at %.0f m altitude, %.1f m/s",
                        handoff.altitude(), handoff.worldVelocity().length()));
        if (handoff == null) return;

        Profile p = Sim.profile(k, "kinetics:reentry_capsule");
        Environment env = ground(k);
        KineticBody body = Sim.body("capsule", p, k,
                new Vec3(0, Sim.y(k, handoff.altitude()), 0),
                new Vec3(handoff.worldVelocity().length(), 0, 0), FlightPhase.DESCENT);
        FlightDirector director = new FlightDirector(k, env, body,
                FlightDirector.Mission.BALLISTIC, new Integrator(k), 11L);
        Sim.Trace trace = Sim.fly(body, env, director, k, 200000, 4, null, null);

        var impact = trace.events.first(KineticEvent.Impact.class);
        boolean sawReentry = trace.events.ofType(KineticEvent.PhaseChange.class).stream()
                .anyMatch(c -> c.to().equals("REENTRY"));

        h.metric("  peak heating rate", String.format("%.4g W/m^2 (threshold %.4g)",
                trace.peakHeating, p.airframe().overheatThreshold()));
        h.metric("  peak dynamic pressure", String.format("%.0f Pa (q_max %.0f)",
                trace.peakDynamicPressure, p.airframe().qMaxPa()));
        h.isTrue("  entered the REENTRY phase", sawReentry,
                trace.events.ofType(KineticEvent.PhaseChange.class).stream()
                        .map(c -> c.from() + "->" + c.to()).toList().toString());
        h.isTrue("  both chutes deployed",
                trace.events.ofType(KineticEvent.ChuteDeployed.class).size() == 2,
                trace.events.ofType(KineticEvent.ChuteDeployed.class).size() + " deployments");
        h.isTrue("  recovered at survivable speed",
                impact != null && impact.velocity().length() < 15.0,
                impact == null ? "never landed"
                        : String.format("touchdown at %.2f m/s after %.1f s",
                                impact.velocity().length(), body.age()));
        out.put("deorbit_to_landing", trace.hash());
    }

    // ---- shared plumbing --------------------------------------------------

    private static Sim.Trace reentry(Constants k, String profileId, double speed, long seed) {
        Profile p = Sim.profile(k, profileId);
        Environment env = ground(k);
        KineticBody body = Sim.body("entry", p, k,
                new Vec3(0, Sim.y(k, k.d("atmosphere.karman_altitude_game")), 0),
                new Vec3(speed, 0, 0), FlightPhase.DESCENT);
        FlightDirector director = new FlightDirector(k, env, body,
                FlightDirector.Mission.BALLISTIC, new Integrator(k), seed);
        return Sim.fly(body, env, director, k, 200000, 4, null, null);
    }

    private record Launch(Profile profile, KineticBody body, Sim.Trace trace,
                          double finalHorizontalFraction, double finalPitchDeg) {}

    private static Launch fly(Constants k, String profileId, long seed) {
        Profile p = Sim.profile(k, profileId);
        // Flat ground, NOT an empty world. A launch flown over nothing cannot fail by flying
        // into the ground, which is exactly how the v0.1.0 gravity-turn bug survived this test.
        Environment env = Environment.overworld(k,
                WorldProbe.flatGround((int) k.d("world.sea_level_y") - 1));
        KineticBody body = Sim.body("rocket", p, k,
                new Vec3(0, Sim.y(k, 0.0), 0), Vec3.ZERO, FlightPhase.RAIL);
        FlightDirector director = new FlightDirector(k, env, body,
                FlightDirector.Mission.LAUNCH, new Integrator(k), seed)
                .downrange(new Vec3(1, 0, 0));
        Sim.Trace trace = Sim.fly(body, env, director, k, 40000, 20, null, null, null,
                b -> b.phase() != FlightPhase.RAIL && b.phase() != FlightPhase.BOOST
                        && b.phase() != FlightPhase.STAGING);

        Vec3 v = body.velocity();
        double speed = v.length();
        double horizontal = Math.hypot(v.x(), v.z());
        double fraction = speed < 1e-9 ? 0.0 : horizontal / speed;
        double pitch = speed < 1e-9 ? 0.0 : Math.toDegrees(Math.atan2(v.y(), horizontal));
        return new Launch(p, body, trace, fraction, pitch);
    }

    private static final class Engagement {
        Sim.Trace trace;
        KineticBody body;
        FlightDirector director;
        KineticEvent.Proximity proximity;
        final List<FlightPhase> reachedPhases = new ArrayList<>();
    }

    /** Constant-acceleration target: the common case. */
    private static Engagement engage(Constants k, String profileId,
                                     Vec3 launchPos, Vec3 launchVel,
                                     Vec3 targetPos, Vec3 targetVel, Vec3 targetAccel,
                                     int maxTicks, long seed, WorldProbe world,
                                     List<Countermeasures.Decoy> decoys) {
        double dt = k.d("world.tick_seconds");
        return engage(k, profileId, launchPos, launchVel, maxTicks, seed, world, decoys,
                tick -> {
                    double t = tick * dt;
                    Vec3 p = targetPos.add(targetVel.scale(t))
                            .add(targetAccel.scale(0.5 * t * t));
                    return new Target("bogey", p, targetVel.add(targetAccel.scale(t)),
                            targetAccel, 5.0, false);
                });
    }

    /** Run a guided engagement against a target whose motion is scripted per tick. */
    private static Engagement engage(Constants k, String profileId,
                                     Vec3 launchPos, Vec3 launchVel,
                                     int maxTicks, long seed, WorldProbe world,
                                     List<Countermeasures.Decoy> decoys,
                                     java.util.function.IntFunction<Target> targetAt) {
        Profile p = Sim.profile(k, profileId);
        Environment env = Environment.overworld(k, world);
        Engagement e = new Engagement();
        e.body = Sim.body("missile", p, k, launchPos, launchVel, FlightPhase.BOOST);
        e.director = new FlightDirector(k, env, e.body, FlightDirector.Mission.GUIDED,
                new Integrator(k), seed);
        e.trace = new Sim.Trace();

        double dt = k.d("world.tick_seconds");
        for (int tick = 0; tick < maxTicks; tick++) {
            if (!e.body.phase().isInWorld()) break;
            if (!e.reachedPhases.contains(e.body.phase())) e.reachedPhases.add(e.body.phase());

            e.director.tick(tick * dt, dt, targetAt.apply(tick), decoys, e.trace.events);
            e.trace.states.add(e.body.snapshot());
            e.trace.ticks = tick + 1;
        }
        if (!e.reachedPhases.contains(e.body.phase())) e.reachedPhases.add(e.body.phase());
        e.trace.finalPhase = e.body.phase();
        e.proximity = e.trace.events.first(KineticEvent.Proximity.class);
        return e;
    }

    private static Environment ground(Constants k) {
        return Environment.overworld(k,
                WorldProbe.flatGround((int) k.d("world.sea_level_y") - 1));
    }

    private static Vec3 origin(Constants k) {
        return new Vec3(0, k.d("world.sea_level_y"), 0);
    }

    private static Vec3 launch(Vec3 heading, double speed, double elevationDeg) {
        double e = Math.toRadians(elevationDeg);
        return heading.normalized().scale(speed * Math.cos(e))
                .add(Vec3.UP.scale(speed * Math.sin(e)));
    }

    private static String sha256(String text) {
        var digest = dev.lilkuzco.kinetics.body.BodyState.newDigest();
        digest.update(text.getBytes(StandardCharsets.UTF_8));
        return dev.lilkuzco.kinetics.body.BodyState.hex(digest.digest());
    }

    private static Map<String, String> loadExpected() {
        Map<String, String> map = new LinkedHashMap<>();
        try (InputStream in = GoldenTests.class.getResourceAsStream(RESOURCE)) {
            if (in == null) return map;
            for (String line : new String(in.readAllBytes(), StandardCharsets.UTF_8).lines()
                    .toList()) {
                String trimmed = line.trim();
                if (trimmed.isEmpty() || trimmed.startsWith("#")) continue;
                int eq = trimmed.indexOf('=');
                if (eq > 0) map.put(trimmed.substring(0, eq).trim(),
                        trimmed.substring(eq + 1).trim());
            }
        } catch (IOException e) {
            throw new IllegalStateException("could not read " + RESOURCE, e);
        }
        return map;
    }

    private static void writeRecord(Path path, Map<String, String> hashes) {
        StringBuilder sb = new StringBuilder();
        sb.append("# Golden trajectory hashes for lilkuzco_kinetics (invariant I7).\n");
        sb.append("# GENERATED — rewrite with: tools/build.sh golden-record\n");
        sb.append("# A hash changing means the trajectory changed. Re-record deliberately,\n");
        sb.append("# never to make a failing test pass.\n");
        for (var e : hashes.entrySet()) {
            sb.append(e.getKey()).append(" = ").append(e.getValue()).append('\n');
        }
        try {
            Files.writeString(path, sb.toString(), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new IllegalStateException("could not write " + path, e);
        }
    }
}
