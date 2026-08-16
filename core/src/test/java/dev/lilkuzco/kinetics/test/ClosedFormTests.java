package dev.lilkuzco.kinetics.test;

import dev.lilkuzco.kinetics.aero.Aerodynamics;
import dev.lilkuzco.kinetics.aero.Compressibility;
import dev.lilkuzco.kinetics.aero.LiftCurve;
import dev.lilkuzco.kinetics.ballistics.BallisticSolver;
import dev.lilkuzco.kinetics.body.KineticBody;
import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.env.Environment;
import dev.lilkuzco.kinetics.env.WorldProbe;
import dev.lilkuzco.kinetics.event.EventSink;
import dev.lilkuzco.kinetics.integrate.ControlCommand;
import dev.lilkuzco.kinetics.integrate.Integrator;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.orbit.GroundTrack;
import dev.lilkuzco.kinetics.orbit.Orbit;
import dev.lilkuzco.kinetics.orbit.OrbitalMechanics;
import dev.lilkuzco.kinetics.orbit.OrbitalRegistry;
import dev.lilkuzco.kinetics.phase.FlightDirector;
import dev.lilkuzco.kinetics.phase.FlightPhase;
import dev.lilkuzco.kinetics.profile.EngineFrame;
import dev.lilkuzco.kinetics.profile.Profile;
import dev.lilkuzco.kinetics.profile.Stage;
import dev.lilkuzco.kinetics.propulsion.Propulsion;
import dev.lilkuzco.kinetics.sensors.Radar;

/**
 * The research made executable: every model checked against the closed form it came from.
 *
 * <p>These are the acceptance criteria. Each one compares something the simulation produced
 * against something written straight out of the published equation, computed independently -
 * a self-check that measured its own output would prove nothing.
 */
public final class ClosedFormTests {

    private ClosedFormTests() {}

    public static void run(Harness h, Constants k) {
        terminalVelocity(h);
        highAltitudeDrop(h, k);
        ballisticRange(h, k);
        tsiolkovsky(h, k);
        altitudeVaryingIsp(h, k);
        orbitalPeriodAndEscape(h, k);
        groundTrackShift(h, k);
        radarFourthRoot(h, k);
        stallCurve(h, k);
        inducedDrag(h, k);
        transonicRise(h, k);
        bluntBodyReentry(h, k);
        ascentAoaLimit(h, k);
    }

    // ---- RB3 --------------------------------------------------------------

    /** Free fall must converge on {@code v_t = sqrt(2mg / (rho C_d A))} within 2%. */
    private static void terminalVelocity(Harness h) {
        h.suite("RB3 — terminal velocity vs closed form");
        Constants k = Sim.uniformAtmosphere();
        h.note("run against a flattened atmosphere so density is constant during the fall; "
                + "terminal velocity is defined for a fixed density.");

        for (String id : new String[]{"kinetics:drop_body", "kinetics:reentry_capsule"}) {
            Profile p = Sim.profile(k, id);
            double mass = p.payloadDryMass();
            double gravity = k.d("gravity.g0");
            double rho = k.d("atmosphere.rho_sea_level");
            double expected = Aerodynamics.terminalVelocity(mass, gravity, rho,
                    p.airframe().cd0(), p.airframe().referenceArea());

            Environment env = Environment.overworld(k, WorldProbe.empty());
            KineticBody body = Sim.body("vt", p, k,
                    new Vec3(0, Sim.y(k, 4000.0), 0), Vec3.ZERO, FlightPhase.DESCENT);
            Integrator integrator = new Integrator(k);
            EventSink sink = EventSink.discarding();

            double dt = k.d("world.tick_seconds");
            for (int tick = 0; tick < 4000; tick++) {
                integrator.step(body, env, ControlCommand.coast(), tick * dt, dt, sink);
            }
            h.near("v_t " + id.substring(id.indexOf(':') + 1),
                    body.speed(), expected, k.d("limits.terminal_velocity_tolerance"), "m/s");
        }
        h.endSuite();
    }

    /**
     * A high drop must ACCELERATE in thin air, then DECELERATE as the air thickens (RB3).
     * The counterintuitive result, and the one that only appears if the atmosphere is right.
     */
    private static void highAltitudeDrop(Harness h, Constants k) {
        h.suite("RB3 — high drop accelerates, then decelerates");
        Profile p = Sim.profile(k, "kinetics:drop_body");
        double karman = k.d("atmosphere.karman_altitude_game");

        Environment env = Environment.overworld(k, WorldProbe.flatGround((int) k.d("world.sea_level_y") - 1));
        KineticBody body = Sim.body("drop", p, k,
                new Vec3(0, Sim.y(k, karman), 0), Vec3.ZERO, FlightPhase.DESCENT);
        FlightDirector director = new FlightDirector(k, env, body,
                FlightDirector.Mission.BALLISTIC, new Integrator(k), 1L);
        Sim.Trace trace = Sim.fly(body, env, director, k, 4000, 1, null, null);

        // Read the impact event, not the final snapshot: on collision the body's velocity is
        // zeroed, so the snapshot records the aftermath rather than the arrival.
        var impact = trace.events.first(dev.lilkuzco.kinetics.event.KineticEvent.Impact.class);
        h.isTrue("body reached the ground", impact != null,
                impact == null ? "no impact event" : "impact at " + impact.position());
        double impactSpeed = impact == null ? 0.0 : impact.velocity().length();
        h.metric("peak speed during the fall", String.format("%.3f m/s", trace.peakSpeed));
        h.metric("speed at impact", String.format("%.3f m/s", impactSpeed));

        double rhoSea = k.d("atmosphere.rho_sea_level");
        double vtSeaLevel = Aerodynamics.terminalVelocity(p.payloadDryMass(), k.d("gravity.g0"),
                rhoSea, p.airframe().cd0(), p.airframe().referenceArea());
        double rhoKarman = rhoSea * Math.exp(-karman / k.d("atmosphere.scale_height"));
        double vtKarman = Aerodynamics.terminalVelocity(p.payloadDryMass(), k.d("gravity.g0"),
                rhoKarman, p.airframe().cd0(), p.airframe().referenceArea());
        h.metric("v_t at the Karman line vs at sea level",
                String.format("%.1f m/s  ->  %.1f m/s", vtKarman, vtSeaLevel));

        h.isTrue("body decelerated after its peak",
                trace.peakSpeed > impactSpeed * 1.05,
                String.format("peak %.3f m/s exceeds impact %.3f m/s by %.1f%%",
                        trace.peakSpeed, impactSpeed,
                        (trace.peakSpeed / impactSpeed - 1.0) * 100.0));
        // It does not fully converge, and should not be asserted to. The body is chasing a
        // terminal velocity that is itself falling as the air thickens - 194 m/s at the Karman
        // line down to 20 m/s at sea level - so it arrives still above the local value. That lag
        // is the physics, not an integration error; what must hold is that it is between the two.
        h.isTrue("impact speed lies between sea-level v_t and the peak",
                impactSpeed > vtSeaLevel && impactSpeed < trace.peakSpeed,
                String.format("v_t %.2f < impact %.2f < peak %.2f m/s (lag above v_t: %.1f%%)",
                        vtSeaLevel, impactSpeed, trace.peakSpeed,
                        (impactSpeed / vtSeaLevel - 1.0) * 100.0));
        h.endSuite();
    }

    // ---- ballistics -------------------------------------------------------

    /** The solver's firing solution, flown by the real integrator, must land within 2%. */
    private static void ballisticRange(Harness h, Constants k) {
        h.suite("Ballistics — solver prediction vs flown trajectory");
        Profile p = Sim.profile(k, "kinetics:mortar_shell");
        int groundY = (int) k.d("world.sea_level_y") - 1;
        Environment env = Environment.overworld(k, WorldProbe.flatGround(groundY));
        BallisticSolver solver = new BallisticSolver(k);

        Vec3 origin = new Vec3(0, k.d("world.sea_level_y"), 0);
        Vec3 heading = new Vec3(1, 0, 0);
        double muzzle = 150.0;

        double best = solver.findMaxRangeElevation(p, env, origin, muzzle, heading,
                k.d("world.sea_level_y"), Vec3.ZERO);
        h.metric("max-range elevation (below 45 deg because of drag)",
                String.format("%.2f deg", best));

        for (double targetRange : new double[]{400.0, 800.0}) {
            for (boolean high : new boolean[]{false, true}) {
                BallisticSolver.Solution solution = solver.solveForRange(p, env, origin, muzzle,
                        heading, targetRange, k.d("world.sea_level_y"), Vec3.ZERO, high);
                if (!solution.converged()) {
                    h.check((high ? "high" : "low") + " arc to " + targetRange + " m",
                            false, solution.note());
                    continue;
                }

                KineticBody body = Sim.body("shell", p, k, origin,
                        launchVelocity(heading, muzzle, solution.elevationDeg()),
                        FlightPhase.DESCENT);
                FlightDirector director = new FlightDirector(k, env, body,
                        FlightDirector.Mission.BALLISTIC, new Integrator(k), 1L);
                Sim.Trace trace = Sim.fly(body, env, director, k, 20000, 4, null, null);

                double flown = Math.hypot(body.position().x() - origin.x(),
                        body.position().z() - origin.z());
                h.near(String.format("%s arc: flown range vs %.0f m solved at %.2f deg",
                                high ? "high" : "low ", targetRange, solution.elevationDeg()),
                        flown, targetRange, k.d("limits.ballistic_range_tolerance"), "m");
            }
        }
        h.endSuite();
    }

    private static Vec3 launchVelocity(Vec3 heading, double speed, double elevationDeg) {
        double e = Math.toRadians(elevationDeg);
        return heading.normalized().scale(speed * Math.cos(e))
                .add(Vec3.UP.scale(speed * Math.sin(e)));
    }

    // ---- RD1, RD4 ---------------------------------------------------------

    /** Achieved delta-v must match Tsiolkovsky within 1%, single-stage and staged. */
    private static void tsiolkovsky(Harness h, Constants k) {
        h.suite("RD1/RD4 — achieved delta-v vs Tsiolkovsky");
        Propulsion propulsion = new Propulsion(k);
        EngineFrame frame = EngineFrame.of(k);
        Environment env = Sim.vacuum(k);
        h.note("flown in vacuum, so vacuum Isp applies throughout and the comparison is against "
                + "the ideal equation rather than an altitude-averaged one.");

        for (String id : new String[]{"kinetics:orbital_rocket_underpowered",
                                      "kinetics:orbital_rocket_2stage"}) {
            Profile p = Sim.profile(k, id);
            double ideal = p.idealDeltaVVacuum(frame);

            KineticBody body = Sim.body("rocket", p, k,
                    new Vec3(0, Sim.y(k, 0.0), 0), Vec3.ZERO, FlightPhase.RAIL);
            FlightDirector director = new FlightDirector(k, env, body,
                    FlightDirector.Mission.LAUNCH, new Integrator(k), 1L);
            // Stop at burnout: achieved delta-v is complete then, and in an empty vacuum world
            // there is no ground to end the flight.
            Sim.Trace trace = Sim.fly(body, env, director, k, 20000, 50, null, null, null,
                    b -> b.phase() != FlightPhase.RAIL && b.phase() != FlightPhase.BOOST
                            && b.phase() != FlightPhase.STAGING);

            String label = id.substring(id.lastIndexOf('_') + 1);
            h.near("delta-v " + label + " (" + p.stages().size() + " stage)",
                    body.achievedDeltaV(), ideal, k.d("limits.tsiolkovsky_tolerance"), "m/s");

            if (p.stages().size() > 1) {
                double[] perStage = propulsion.stageDeltaV(p);
                h.metric("  per-stage ideal delta-v",
                        String.format("%.1f + %.1f = %.1f m/s", perStage[0], perStage[1], ideal));
                h.isTrue("  staging event fired",
                        trace.events.has(dev.lilkuzco.kinetics.event.KineticEvent.Staging.class),
                        trace.events.ofType(
                                dev.lilkuzco.kinetics.event.KineticEvent.Staging.class).size()
                                + " staging events");
            }
            h.isTrue("  dry mass is the floor for " + label,
                    Math.abs(body.mass() - p.payloadDryMass()) < 1e-6,
                    String.format("final mass %.6f kg, payload dry mass %.6f kg",
                            body.mass(), p.payloadDryMass()));
        }
        h.endSuite();
    }

    /** Effective Isp at the pad and at the Karman line must match the RD2b interpolation. */
    private static void altitudeVaryingIsp(Harness h, Constants k) {
        h.suite("RD2b — altitude-varying specific impulse");
        Profile p = Sim.profile(k, "kinetics:orbital_rocket_2stage");
        Stage stage = p.stages().get(0);
        Environment env = Environment.overworld(k);

        double padRatio = env.pressureRatioAt(Sim.y(k, 0.0));
        double karmanRatio = env.pressureRatioAt(Sim.y(k, k.d("atmosphere.karman_altitude_game")));

        h.near("pressure ratio at the pad", padRatio, 1.0, 1e-9, "");
        h.metric("pressure ratio at the Karman line", String.format("%.6f", karmanRatio));

        double ispPad = stage.effectiveIsp(padRatio);
        double ispKarman = stage.effectiveIsp(karmanRatio);
        double expectedKarman = stage.ispVacuum()
                + (stage.ispSeaLevel() - stage.ispVacuum()) * karmanRatio;

        h.near("Isp at the pad equals sea-level Isp", ispPad, stage.ispSeaLevel(), 1e-9, "s");
        h.near("Isp at the Karman line matches the interpolation",
                ispKarman, expectedKarman, 1e-9, "s");
        h.isTrue("Isp rises with altitude", ispKarman > ispPad,
                String.format("%.3f s at the pad -> %.3f s at the Karman line (vacuum %.1f s)",
                        ispPad, ispKarman, stage.ispVacuum()));

        EngineFrame frame = EngineFrame.of(k);
        h.metric("sea-level thrust vs vacuum thrust",
                String.format("%.0f N -> %.0f N (+%.1f%%)",
                        stage.effectiveThrust(1.0, frame), stage.effectiveThrust(0.0, frame),
                        (stage.effectiveThrust(0.0, frame) / stage.effectiveThrust(1.0, frame)
                                - 1.0) * 100.0));
        h.endSuite();
    }

    // ---- RE1, RE2, RE5b ---------------------------------------------------

    private static void orbitalPeriodAndEscape(Harness h, Constants k) {
        h.suite("RE1/RE2 — orbital period and escape velocity");
        OrbitalMechanics m = new OrbitalMechanics(k);
        double r = m.radiusForAltitude(k.d("orbit.reference_orbit_altitude"));

        h.near("period at the reference altitude", m.period(r), k.d("world.day_seconds"),
                k.d("limits.orbital_period_tolerance"), "s");

        double vCirc = m.circularVelocity(r);
        h.near("circular velocity matches 2*pi*r/T", vCirc, 2.0 * Math.PI * r / m.period(r),
                1e-9, "m/s");
        h.near("escape velocity is sqrt(2) x circular",
                m.escapeVelocity(r), Math.sqrt(2.0) * vCirc, 1e-12, "m/s");
        h.near("vis-viva at r = a reduces to the circular case",
                m.visViva(r, r), vCirc, 1e-12, "m/s");
        h.near("mu is derived from g0 and R", m.mu(),
                k.d("gravity.g0") * m.planetRadius() * m.planetRadius(), 1e-6, "m^3/s^2");
        h.endSuite();
    }

    /** Successive ground tracks must shift west by exactly (T_orbit/T_day)*360 degrees. */
    private static void groundTrackShift(Harness h, Constants k) {
        h.suite("RE5b — ground-track shift in the rotating frame");
        OrbitalRegistry registry = new OrbitalRegistry(k);
        OrbitalMechanics m = registry.mechanics();

        // Deliberately NOT the reference altitude: there T_orbit equals T_day and the shift is
        // a degenerate 360 degrees. A different altitude gives a real, checkable drift.
        double altitude = 40000.0;
        Orbit orbit = Orbit.circular("gt", m, 0.0, altitude, 60.0, 0.0, 0.0);
        registry.register(orbit, dev.lilkuzco.kinetics.orbit.Attitude.threeAxis(
                dev.lilkuzco.kinetics.math.Quat.IDENTITY, 5.0));

        double period = m.period(m.radiusForAltitude(altitude));
        double expectedShift = m.groundTrackShiftPerOrbit(m.radiusForAltitude(altitude));
        h.metric("orbital period at 40,000 m", String.format("%.3f s", period));
        h.metric("expected shift per orbit",
                String.format("%.4f deg  (mod 360 = %.4f deg west)",
                        expectedShift, GroundTrack.normalizeLongitude(-expectedShift)));

        // Sample the longitude at the same point in the orbit on successive revolutions.
        for (int rev = 1; rev <= 3; rev++) {
            GroundTrack a = registry.stateOf(orbit, (rev - 1) * period).groundTrack();
            GroundTrack b = registry.stateOf(orbit, rev * period).groundTrack();
            double measured = GroundTrack.normalizeLongitude(b.longitudeDeg() - a.longitudeDeg());
            double expected = GroundTrack.normalizeLongitude(-expectedShift);
            h.near("revolution " + rev + " longitude shift", measured, expected, 1e-6, "deg");
        }

        // And confirm the degenerate case is genuinely degenerate at the reference altitude.
        double refShift = m.groundTrackShiftPerOrbit(
                m.radiusForAltitude(k.d("orbit.reference_orbit_altitude")));
        h.isTrue("reference orbit repeats its ground track daily",
                Math.abs(GroundTrack.normalizeLongitude(refShift)) < 0.05,
                String.format("shift %.6f deg, i.e. %.6f deg from repeating",
                        refShift, GroundTrack.normalizeLongitude(refShift)));
        h.endSuite();
    }

    // ---- RF1 --------------------------------------------------------------

    private static void radarFourthRoot(Harness h, Constants k) {
        h.suite("RF1 — radar range scales as the fourth root of RCS");
        Radar radar = new Radar(k);
        double reference = radar.detectionRange(radar.referenceRcs());

        double hundredthRcs = radar.detectionRange(radar.referenceRcs() / 100.0);
        h.near("100x smaller RCS shortens range by 3.162x",
                reference / hundredthRcs, Math.pow(100.0, 0.25), 1e-9, "x");

        double sixteenth = radar.detectionRange(radar.referenceRcs() / 16.0);
        h.near("16x smaller RCS shortens range by exactly 2x",
                reference / sixteenth, 2.0, 1e-9, "x");

        h.metric("stealth vs fighter detection range",
                String.format("%.2f m vs %.2f m — a %.0fx RCS difference buys only %.2fx",
                        radar.detectionRange(k.d("sensors.rcs_bins.stealth")),
                        radar.detectionRange(k.d("sensors.rcs_bins.fighter")),
                        k.d("sensors.rcs_bins.fighter") / k.d("sensors.rcs_bins.stealth"),
                        radar.detectionRange(k.d("sensors.rcs_bins.fighter"))
                                / radar.detectionRange(k.d("sensors.rcs_bins.stealth"))));
        h.endSuite();
    }

    // ---- RB4, RB4b, RB5 ---------------------------------------------------

    private static void stallCurve(Harness h, Constants k) {
        h.suite("RB4 — lift peaks at the stall angle, then collapses");
        LiftCurve curve = LiftCurve.standard(k);

        double peakAoa = 0.0;
        double peakCl = -1.0;
        for (double aoa = 0.0; aoa <= 40.0; aoa += 0.1) {
            double cl = curve.coefficientAt(aoa);
            if (cl > peakCl) { peakCl = cl; peakAoa = aoa; }
        }
        h.near("angle of peak lift", peakAoa, k.d("aerodynamics.default_stall_aoa_deg"),
                0.02, "deg");
        h.near("C_L,max", peakCl, curve.clMax(), 1e-9, "");

        double clAt25 = curve.coefficientAt(25.0);
        double clAt40 = curve.coefficientAt(40.0);
        h.isTrue("lift collapses past the stall",
                clAt25 < peakCl * 0.6,
                String.format("C_L falls from %.4f at %.1f deg to %.4f at 25 deg",
                        peakCl, peakAoa, clAt25));
        h.isTrue("lift keeps falling and stays bounded",
                clAt40 < clAt25 && clAt40 >= 0.0,
                String.format("C_L at 40 deg is %.4f", clAt40));
        h.near("no lift broadside to the flow", curve.coefficientAt(90.0), 0.0, 1e-12, "");
        h.isTrue("lift is antisymmetric in angle of attack",
                Math.abs(curve.coefficientAt(-10.0) + curve.coefficientAt(10.0)) < 1e-12,
                String.format("C_L(-10) = %.6f, C_L(+10) = %.6f",
                        curve.coefficientAt(-10.0), curve.coefficientAt(10.0)));
        h.endSuite();
    }

    /** A sustained maximum-lift turn must bleed speed: induced drag is not free (RB4b/I3). */
    private static void inducedDrag(Harness h, Constants k) {
        h.suite("RB4b — a sustained hard turn bleeds speed");
        Profile p = Sim.profile(k, "kinetics:glider");
        Environment env = Environment.overworld(k, WorldProbe.empty());
        double startSpeed = 120.0;
        double altitude = 150.0;

        double[] straight = flyGlider(k, env, p, startSpeed, altitude, false);
        double[] turning = flyGlider(k, env, p, startSpeed, altitude, true);
        double startEnergy = 0.5 * startSpeed * startSpeed + k.d("gravity.g0") * altitude;

        // Compare mechanical energy, not speed. Both gliders trade altitude for speed as they
        // descend, so a speed comparison alone conflates the dive with the drag. Energy is what
        // induced drag actually removes, and it is the same quantity invariant I3 watches.
        h.metric("specific energy after 6 s straight",
                String.format("%.1f J/kg (lost %.1f)", straight[2], startEnergy - straight[2]));
        h.metric("specific energy after 6 s turning",
                String.format("%.1f J/kg (lost %.1f)", turning[2], startEnergy - turning[2]));
        h.metric("speeds", String.format("straight %.3f m/s, turning %.3f m/s",
                straight[0], turning[0]));
        h.isTrue("the turning glider is slower",
                turning[0] < straight[0],
                String.format("%.3f m/s against %.3f m/s — a %.3f m/s penalty for turning",
                        turning[0], straight[0], straight[0] - turning[0]));
        h.isTrue("the turning glider lost more energy to drag",
                (startEnergy - turning[2]) > (startEnergy - straight[2]),
                String.format("turning lost %.1f J/kg against %.1f J/kg — %.1fx more",
                        startEnergy - turning[2], startEnergy - straight[2],
                        (startEnergy - turning[2]) / Math.max(startEnergy - straight[2], 1e-9)));

        // And the polar itself: doubling C_L must quadruple the induced term.
        var polar = p.airframe().aerodynamics(k).dragPolar();
        double induced1 = polar.inducedComponent(0.5);
        double induced2 = polar.inducedComponent(1.0);
        h.near("induced drag is quadratic in C_L", induced2 / induced1, 4.0, 1e-9, "x");
        h.endSuite();
    }

    /** @return {final speed, final altitude, final specific mechanical energy} */
    private static double[] flyGlider(Constants k, Environment env, Profile p,
                                      double startSpeed, double altitude, boolean turning) {
        KineticBody body = Sim.body(turning ? "turn" : "straight", p, k,
                new Vec3(0, Sim.y(k, altitude), 0), new Vec3(startSpeed, 0, 0),
                FlightPhase.MIDCOURSE);
        Integrator integrator = new Integrator(k);
        EventSink sink = EventSink.discarding();
        double dt = k.d("world.tick_seconds");

        for (int tick = 0; tick < 120; tick++) {
            ControlCommand command;
            if (turning) {
                // Demand far more lateral acceleration than the wing can make, so the airframe
                // sits at C_L,max for the whole run - a genuinely sustained maximum-lift turn.
                Vec3 lateral = body.velocity().cross(Vec3.UP).normalized().scale(500.0);
                command = ControlCommand.accelerate(lateral, 0.0);
            } else {
                command = ControlCommand.pointAt(body.velocity(), 0.0);
            }
            integrator.step(body, env, command, tick * dt, dt, sink);
        }
        double finalAltitude = env.altitudeOf(body.position().y());
        return new double[]{
                body.speed(),
                finalAltitude,
                body.specificEnergy(env.gravity(), finalAltitude)};
    }

    private static void transonicRise(Harness h, Constants k) {
        h.suite("RB5 — transonic drag rise");
        Compressibility c = new Compressibility(k);

        h.near("subsonic multiplier is 1", c.dragMultiplier(0.5), 1.0, 1e-12, "x");
        h.near("still 1 at the drag-divergence Mach",
                c.dragMultiplier(c.divergenceMach()), 1.0, 1e-12, "x");

        double peak = 0.0;
        double peakMach = 0.0;
        for (double m = 0.0; m <= 4.0; m += 0.001) {
            double mult = c.dragMultiplier(m);
            if (mult > peak) { peak = mult; peakMach = m; }
        }
        h.near("peak drag multiplier occurs near Mach 1",
                peakMach, k.d("aerodynamics.transonic.peak_mach"), 0.01, "Mach");
        h.near("peak multiplier", peak, k.d("aerodynamics.transonic.peak_multiplier"), 1e-9, "x");
        h.isTrue("drag eases off supersonically but stays above subsonic",
                c.dragMultiplier(3.5) < peak && c.dragMultiplier(3.5) > 1.0,
                String.format("Mach 1.05: %.4fx, Mach 2: %.4fx, Mach 3.5: %.4fx",
                        c.dragMultiplier(1.05), c.dragMultiplier(2.0), c.dragMultiplier(3.5)));
        h.isTrue("the multiplier is finite everywhere (no Prandtl-Glauert singularity)",
                Double.isFinite(c.dragMultiplier(1.0)) && Double.isFinite(c.dragMultiplier(0.999)),
                String.format("at Mach 1.000: %.6f, at Mach 0.999: %.6f",
                        c.dragMultiplier(1.0), c.dragMultiplier(0.999)));
        h.endSuite();
    }

    /**
     * RD6 - the ascent angle-of-attack limit constrains the PRODUCT q*alpha, so it must relax as
     * dynamic pressure falls. The regression guard for the liftoff crash cosmos found.
     */
    private static void ascentAoaLimit(Harness h, Constants k) {
        h.suite("RD6 — the q*alpha limit relaxes as dynamic pressure falls");
        var turn = dev.lilkuzco.kinetics.propulsion.GravityTurn.standard(k);
        double qMax = k.d("limits.q_max_default");
        double limit = k.d("limits.ascent_aoa_limit_deg");

        // On the pad: barely moving, essentially no q, and the commanded attitude must be
        // obeyed rather than clamped to whatever direction the vehicle is drifting.
        Vec3 drifting = new Vec3(0, -0.5, 0);   // one substep of gravity, before thrust builds
        Vec3 commanded = turn.desiredDirection(2.0, new Vec3(1, 0, 0), drifting, 0.4, qMax);
        double pitchDeg = Math.toDegrees(Math.asin(Math.max(-1.0, Math.min(1.0, commanded.y()))));
        h.greater("a vehicle at 0.5 m/s still points where it was told", pitchDeg, 80.0, "deg");
        h.isTrue("  and is NOT clamped toward its own downward drift",
                commanded.y() > 0.0,
                String.format("commanded %s — a clamp here is what flew rockets into the ground",
                        commanded));

        // At max-Q the limit binds hard, which is the whole point of having it.
        h.isTrue("at max-Q the limit binds",
                turn.exceedsAoaLimit(new Vec3(300, 0, 0), new Vec3(0, 1, 0), qMax, qMax),
                String.format("a 90 deg demand at q_max exceeds the %.0f deg allowance", limit));
        h.isTrue("well below max-Q it does not",
                !turn.exceedsAoaLimit(new Vec3(20, 0, 0), new Vec3(0, 1, 0), qMax / 200.0, qMax),
                "the allowance scales as q_max/q, so thin air permits manoeuvre");
        h.endSuite();
    }

    // ---- RE7 --------------------------------------------------------------

    /** The blunt-body insight: higher C_d gives LOWER peak heating (RE7). */
    private static void bluntBodyReentry(Harness h, Constants k) {
        h.suite("RE7 — blunt bodies reenter cooler than slender ones");
        double entrySpeed = 1200.0;
        double entryAltitude = k.d("atmosphere.karman_altitude_game");
        int groundY = (int) k.d("world.sea_level_y") - 1;
        Environment env = Environment.overworld(k, WorldProbe.flatGround(groundY));

        Profile blunt = Sim.profile(k, "kinetics:reentry_capsule");
        Profile slender = Sim.profile(k, "kinetics:reentry_slender");
        double bluntBeta = blunt.payloadDryMass()
                / (blunt.airframe().cd0() * blunt.airframe().referenceArea());
        double slenderBeta = slender.payloadDryMass()
                / (slender.airframe().cd0() * slender.airframe().referenceArea());

        double bluntPeak = reentryPeak(k, env, blunt, entrySpeed, entryAltitude);
        double slenderPeak = reentryPeak(k, env, slender, entrySpeed, entryAltitude);

        h.metric("ballistic coefficient beta = m/(C_d A)",
                String.format("blunt %.1f kg/m^2, slender %.1f kg/m^2", bluntBeta, slenderBeta));
        h.metric("peak heating rate",
                String.format("blunt %.4g W/m^2, slender %.4g W/m^2", bluntPeak, slenderPeak));
        h.isTrue("the blunt body peaks cooler",
                bluntPeak < slenderPeak,
                String.format("blunt %.4g W/m^2 < slender %.4g W/m^2 (a factor of %.2f)",
                        bluntPeak, slenderPeak, slenderPeak / bluntPeak));
        h.endSuite();
    }

    private static double reentryPeak(Constants k, Environment env, Profile p,
                                      double speed, double altitude) {
        KineticBody body = Sim.body("entry", p, k,
                new Vec3(0, Sim.y(k, altitude), 0), new Vec3(speed, 0, 0), FlightPhase.DESCENT);
        FlightDirector director = new FlightDirector(k, env, body,
                FlightDirector.Mission.BALLISTIC, new Integrator(k), 1L);
        Sim.Trace trace = Sim.fly(body, env, director, k, 40000, 20, null, null);
        return trace.peakHeating;
    }
}
