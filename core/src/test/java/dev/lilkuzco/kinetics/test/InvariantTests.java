package dev.lilkuzco.kinetics.test;

import dev.lilkuzco.kinetics.body.BodyState;
import dev.lilkuzco.kinetics.body.KineticBody;
import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.constants.ScaleAudit;
import dev.lilkuzco.kinetics.env.Environment;
import dev.lilkuzco.kinetics.env.WorldProbe;
import dev.lilkuzco.kinetics.event.EventSink;
import dev.lilkuzco.kinetics.event.KineticEvent;
import dev.lilkuzco.kinetics.integrate.ControlCommand;
import dev.lilkuzco.kinetics.integrate.Integrator;
import dev.lilkuzco.kinetics.integrate.SweptCollision;
import dev.lilkuzco.kinetics.math.Quat;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.phase.FlightDirector;
import dev.lilkuzco.kinetics.phase.FlightPhase;
import dev.lilkuzco.kinetics.phase.PhaseMachine;
import dev.lilkuzco.kinetics.profile.Profile;
import dev.lilkuzco.kinetics.profile.ProfileLoader;
import dev.lilkuzco.kinetics.util.Json;

import java.lang.reflect.RecordComponent;
import java.util.List;
import java.util.Map;

/**
 * The physics constitution as CI gates (Section 0, I1-I12).
 *
 * <p>Several invariants are enforced by construction and cannot be violated by any code path;
 * those are tested structurally rather than behaviourally. I10 in particular is checked by
 * reflecting over the sealed event hierarchy - the guarantee is that no event can carry damage,
 * and the way to verify that is to look at every event there is.
 */
public final class InvariantTests {

    private InvariantTests() {}

    public static void run(Harness h, Constants k) {
        i1Continuity(h, k);
        i2BoundedForces(h, k);
        i3EnergyHonesty(h, k);
        i4MassAccounting(h, k);
        i5DragSanity(h, k);
        i6TurnAuthority(h, k);
        i7Determinism(h, k);
        i9NoMagicNumbers(h, k);
        i10SingleDamageDoor(h, k);
        i11ScaleAudit(h, k);
        i12QuaternionSanity(h, k);
        phaseMachine(h, k);
    }

    // ---- I1 ---------------------------------------------------------------

    private static void i1Continuity(Harness h, Constants k) {
        h.suite("I1 — continuity: no tunnelling, no NaN");

        // A one-block wall, and a body moving fast enough to cross it many times over in a
        // single substep. Point sampling would miss it; swept traversal cannot.
        WorldProbe wall = (x, y, z) -> x == 500;
        SweptCollision.Hit hit = SweptCollision.cast(wall,
                new Vec3(0, 70, 0), new Vec3(1000, 70, 0));
        h.isTrue("swept cast finds a 1-block wall 1000 m away",
                hit != null && hit.blockX() == 500,
                hit == null ? "MISSED — the body tunnelled" : "hit block x=" + hit.blockX()
                        + " at " + String.format("%.3f m", hit.distance()));

        // Same wall, same step, at every angle: none may pass through.
        int misses = 0;
        for (int i = 0; i < 360; i++) {
            double a = Math.toRadians(i);
            Vec3 from = new Vec3(499.5, 70.5, 0.5);
            Vec3 to = from.add(new Vec3(Math.cos(a) * 50.0, Math.sin(a) * 50.0, 0));
            boolean crossesWall = to.x() >= 500.0;
            SweptCollision.Hit hh = SweptCollision.cast(wall, from, to);
            if (crossesWall && hh == null) misses++;
        }
        h.isTrue("no tunnelling across 360 approach angles", misses == 0,
                misses + " missed crossings out of 360");

        // A long, fast, fully simulated flight must stay finite at every recorded state.
        Profile p = Sim.profile(k, "kinetics:reentry_capsule");
        Environment env = Environment.overworld(k,
                WorldProbe.flatGround((int) k.d("world.sea_level_y") - 1));
        KineticBody body = Sim.body("finite", p, k,
                new Vec3(0, Sim.y(k, k.d("atmosphere.karman_altitude_game")), 0),
                new Vec3(1500, 0, 0), FlightPhase.DESCENT);
        FlightDirector director = new FlightDirector(k, env, body,
                FlightDirector.Mission.BALLISTIC, new Integrator(k), 7L);
        Sim.Trace trace = Sim.fly(body, env, director, k, 40000, 1, null, null);

        long nonFinite = trace.states.stream().filter(s -> !s.isFinite()).count();
        h.isTrue("every state of a " + trace.states.size() + "-sample reentry is finite",
                nonFinite == 0, nonFinite + " non-finite states");
        h.endSuite();
    }

    // ---- I2 ---------------------------------------------------------------

    private static void i2BoundedForces(Harness h, Constants k) {
        h.suite("I2 — commanded acceleration is clamped to the g-limit");
        Profile p = Sim.profile(k, "kinetics:interceptor");
        Environment env = Environment.overworld(k, WorldProbe.empty());
        Integrator integrator = new Integrator(k);
        double hardCap = k.d("limits.g_limit_hard_cap") * k.d("gravity.g0");
        double profileLimit = p.airframe().gLimitG() * k.d("gravity.g0");

        KineticBody body = Sim.body("clamp", p, k,
                new Vec3(0, Sim.y(k, 100.0), 0), new Vec3(600, 0, 0), FlightPhase.MIDCOURSE);
        double dt = k.d("world.tick_seconds");
        double worstAccel = 0.0;

        for (int tick = 0; tick < 40; tick++) {
            Vec3 before = body.velocity();
            // Demand 200 g perpendicular to flight - far beyond anything the airframe can do.
            Vec3 absurd = new Vec3(0, 1, 0).scale(200.0 * k.d("gravity.g0"));
            integrator.step(body, env, ControlCommand.accelerate(absurd, 0.0),
                    tick * dt, dt, EventSink.discarding());
            // Back out the achieved acceleration, less gravity, which is not clamped.
            Vec3 achieved = body.velocity().sub(before).scale(1.0 / dt).sub(env.gravityVector());
            worstAccel = Math.max(worstAccel, achieved.length());
        }

        h.metric("commanded", String.format("%.1f m/s^2 (200 g)", 200.0 * k.d("gravity.g0")));
        h.metric("profile g-limit / hard cap",
                String.format("%.1f / %.1f m/s^2 (%.0f g / %.0f g)",
                        profileLimit, hardCap, p.airframe().gLimitG(),
                        k.d("limits.g_limit_hard_cap")));
        h.less("achieved acceleration stays under the hard cap", worstAccel, hardCap * 1.001,
                "m/s^2");
        h.isTrue("a 200 g demand is refused",
                worstAccel < 200.0 * k.d("gravity.g0") * 0.5,
                String.format("peak achieved %.1f m/s^2 (%.1f g)",
                        worstAccel, worstAccel / k.d("gravity.g0")));

        // And the loader clamps an over-cap profile rather than accepting it.
        ProfileLoader loader = new ProfileLoader(k);
        Profile greedy = loader.load("""
                {"id":"test:greedy","payload_dry_mass":100.0,
                 "airframe":{"reference_area":1.0,"cd0":0.3,"g_limit":500.0}}""");
        h.near("a 500 g profile is clamped at load", greedy.airframe().gLimitG(),
                k.d("limits.g_limit_hard_cap"), 1e-12, "g");
        h.isTrue("and the clamp is reported as a warning",
                loader.warnings().stream().anyMatch(w -> w.contains("hard cap")),
                loader.warnings().isEmpty() ? "no warnings" : loader.warnings().get(0));
        h.endSuite();
    }

    // ---- I3 ---------------------------------------------------------------

    private static void i3EnergyHonesty(Harness h, Constants k) {
        h.suite("I3 — unpowered bodies only lose energy");
        Environment env = Environment.overworld(k, WorldProbe.empty());
        Integrator integrator = new Integrator(k);

        for (String id : new String[]{"kinetics:mortar_shell", "kinetics:glider",
                                      "kinetics:drop_body"}) {
            Profile p = Sim.profile(k, id);
            KineticBody body = Sim.body("energy", p, k,
                    new Vec3(0, Sim.y(k, 220.0), 0), new Vec3(90, 40, 15), FlightPhase.MIDCOURSE);
            double dt = k.d("world.tick_seconds");
            double previous = body.specificEnergy(env.gravity(),
                    env.altitudeOf(body.position().y()));
            double worstGain = 0.0;

            for (int tick = 0; tick < 400; tick++) {
                // Bank hard the whole way: lift is doing its most, and must still do no work.
                Vec3 lateral = body.velocity().cross(Vec3.UP).normalized().scale(300.0);
                integrator.step(body, env, ControlCommand.accelerate(lateral, 0.0),
                        tick * dt, dt, EventSink.discarding());
                double now = body.specificEnergy(env.gravity(),
                        env.altitudeOf(body.position().y()));
                worstGain = Math.max(worstGain, now - previous);
                previous = now;
            }
            h.isTrue("energy never rose while manoeuvring: " + id.substring(id.indexOf(':') + 1),
                    worstGain <= 0.0,
                    String.format("largest single-tick change %+.6g J/kg", worstGain));
        }
        h.note("the integrator itself throws on a gain above tolerance, so reaching this line "
                + "at all means no step created energy.");
        h.endSuite();
    }

    // ---- I4 ---------------------------------------------------------------

    private static void i4MassAccounting(Harness h, Constants k) {
        h.suite("I4 — mass only falls by burning declared fuel");
        Profile p = Sim.profile(k, "kinetics:orbital_rocket_2stage");
        Environment env = Sim.vacuum(k);
        KineticBody body = Sim.body("mass", p, k,
                new Vec3(0, Sim.y(k, 0.0), 0), Vec3.ZERO, FlightPhase.RAIL);
        FlightDirector director = new FlightDirector(k, env, body,
                FlightDirector.Mission.LAUNCH, new Integrator(k), 3L);

        double[] previous = {p.wetMass()};
        boolean[] rose = {false};
        Sim.fly(body, env, director, k, 20000, 1, null, null, (tick, b) -> {
            if (b.mass() > previous[0] + 1e-9) rose[0] = true;
            previous[0] = b.mass();
        }, b -> b.phase() != FlightPhase.RAIL && b.phase() != FlightPhase.BOOST
                && b.phase() != FlightPhase.STAGING);

        h.isTrue("mass never increased", !rose[0], "monotonically non-increasing across the burn");
        h.near("final mass equals the dry-mass floor", body.mass(), p.payloadDryMass(),
                1e-12, "kg");
        h.isTrue("no fuel remains", body.stageFuel() == 0.0,
                String.format("%.9f kg remaining", body.stageFuel()));

        // Burn time is derived from the fuel load, never declared - so the two must agree.
        var frame = dev.lilkuzco.kinetics.profile.EngineFrame.of(k);
        double declaredBurn = p.stages().get(0).burnTime(frame) + p.stages().get(1).burnTime(frame);
        h.metric("total burn time derived from the fuel load",
                String.format("%.3f s (stage 1 %.2f s, stage 2 %.2f s)",
                        declaredBurn, p.stages().get(0).burnTime(frame),
                        p.stages().get(1).burnTime(frame)));
        h.near("flight duration matches the derived burn time", body.age(), declaredBurn,
                0.01, "s");
        h.endSuite();
    }

    // ---- I5 ---------------------------------------------------------------

    private static void i5DragSanity(Harness h, Constants k) {
        h.suite("I5 — drag opposes the airspeed vector");
        Profile p = Sim.profile(k, "kinetics:glider");
        var aero = p.airframe().aerodynamics(k);
        int violations = 0;
        double worstAlignment = Double.NEGATIVE_INFINITY;

        // Sweep speed, altitude and angle of attack, including deep into the stall.
        for (double speed = 5.0; speed <= 900.0; speed += 17.0) {
            for (double altitude = 0.0; altitude <= 250.0; altitude += 25.0) {
                for (double aoa = -80.0; aoa <= 80.0; aoa += 7.0) {
                    Vec3 airspeed = new Vec3(speed, 0, 0);
                    Vec3 forward = new Vec3(Math.cos(Math.toRadians(aoa)),
                            Math.sin(Math.toRadians(aoa)), 0);
                    double rho = k.d("atmosphere.rho_sea_level")
                            * Math.exp(-altitude / k.d("atmosphere.scale_height"));
                    var res = aero.compute(airspeed, forward, rho,
                            speed / k.d("atmosphere.speed_of_sound_sea_level"),
                            p.airframe().referenceArea(), p.airframe().wingArea());
                    double alignment = res.dragForce().dot(airspeed);
                    if (alignment > 0.0) violations++;
                    worstAlignment = Math.max(worstAlignment, alignment);
                    // Lift must do no work: exactly perpendicular to the airflow.
                    double liftWork = Math.abs(res.liftForce().dot(airspeed));
                    if (liftWork > 1e-6 * Math.max(1.0, res.liftForce().length() * speed)) {
                        violations++;
                    }
                }
            }
        }
        h.isTrue("drag opposed airspeed and lift did no work, across 5,000+ states",
                violations == 0,
                violations + " violations; worst drag-airspeed dot product "
                        + String.format("%.3g", worstAlignment));

        // C_L is bounded no matter how absurd the angle.
        double maxCl = 0.0;
        for (double aoa = -720.0; aoa <= 720.0; aoa += 0.5) {
            maxCl = Math.max(maxCl, Math.abs(aero.liftCurve().coefficientAt(aoa)));
        }
        h.near("C_L is bounded by C_L,max even at absurd angles", maxCl,
                aero.liftCurve().clMax(), 1e-9, "");
        h.endSuite();
    }

    // ---- I6 ---------------------------------------------------------------

    private static void i6TurnAuthority(Harness h, Constants k) {
        h.suite("I6 — turn authority is speed-dependent");
        Profile p = Sim.profile(k, "kinetics:interceptor");
        Environment env = Environment.overworld(k, WorldProbe.empty());
        Integrator integrator = new Integrator(k);
        double dt = k.d("world.tick_seconds");

        double[] speeds = {40.0, 100.0, 250.0, 600.0};
        double[] achieved = new double[speeds.length];

        for (int i = 0; i < speeds.length; i++) {
            KineticBody body = Sim.body("turn" + i, p, k,
                    new Vec3(0, Sim.y(k, 50.0), 0), new Vec3(speeds[i], 0, 0),
                    FlightPhase.MIDCOURSE);
            Vec3 before = body.velocity();
            Vec3 demand = new Vec3(0, 1, 0).scale(60.0 * k.d("gravity.g0"));
            integrator.step(body, env, ControlCommand.accelerate(demand, 0.0), 0.0, dt,
                    EventSink.discarding());
            achieved[i] = body.velocity().sub(before).scale(1.0 / dt)
                    .sub(env.gravityVector()).length();
            h.metric(String.format("at %6.1f m/s", speeds[i]),
                    String.format("%7.2f m/s^2 available (%.2f g)",
                            achieved[i], achieved[i] / k.d("gravity.g0")));
        }

        h.isTrue("a slow body cannot turn like a fast one",
                achieved[0] < achieved[3] * 0.25,
                String.format("%.2f g at 40 m/s against %.2f g at 600 m/s",
                        achieved[0] / k.d("gravity.g0"), achieved[3] / k.d("gravity.g0")));
        h.isTrue("turn authority rises monotonically with speed",
                achieved[0] < achieved[1] && achieved[1] < achieved[2],
                "stall-limited at low speed, structure-limited at high speed");
        h.less("even at 600 m/s the g-limit still binds", achieved[3],
                p.airframe().gLimitG() * k.d("gravity.g0") * 1.001, "m/s^2");

        // An instant reversal is impossible: the slew rate bounds attitude change.
        KineticBody body = Sim.body("reverse", p, k,
                new Vec3(0, Sim.y(k, 50.0), 0), new Vec3(300, 0, 0), FlightPhase.MIDCOURSE);
        Quat start = body.orientation();
        integrator.step(body, env, ControlCommand.pointAt(new Vec3(-1, 0, 0), 0.0), 0.0, dt,
                EventSink.discarding());
        double turnedDeg = Math.toDegrees(start.angleTo(body.orientation()));
        h.less("a commanded 180 deg reversal turns at most the slew rate in one tick",
                turnedDeg, p.maxSlewRateDeg() * dt * 1.001 + 1e-9, "deg");
        h.endSuite();
    }

    // ---- I7 ---------------------------------------------------------------

    private static void i7Determinism(Harness h, Constants k) {
        h.suite("I7 — identical inputs give bit-identical trajectories");

        String first = null;
        for (int run = 0; run < 3; run++) {
            String hash = deterministicScenario(k);
            if (first == null) first = hash;
            else h.equalStrings("run " + (run + 1) + " matches run 1", hash, first);
        }
        h.metric("trajectory hash", first);

        // A different seed must give a different answer, or the seed is not being used.
        h.isTrue("a different seed produces a different trajectory",
                !deterministicScenario(k).equals(seededScenario(k, 99L)),
                "seeded dispersion actually depends on the seed");
        h.endSuite();
    }

    private static String deterministicScenario(Constants k) { return seededScenario(k, 42L); }

    private static String seededScenario(Constants k, long seed) {
        Profile p = Sim.profile(k, "kinetics:mortar_shell");
        Environment env = Environment.overworld(k,
                WorldProbe.flatGround((int) k.d("world.sea_level_y") - 1))
                .withWind(dev.lilkuzco.kinetics.env.WindField.seeded(k, seed));
        var dispersion = new dev.lilkuzco.kinetics.ballistics.Dispersion(k);
        Vec3 origin = new Vec3(0, k.d("world.sea_level_y"), 0);
        Vec3 aim = dispersion.disperseAimPoint(origin, new Vec3(500, 63, 0), p.cep(), seed, 3);

        KineticBody body = Sim.body("det", p, k, origin,
                aim.sub(origin).normalized().scale(150.0).add(Vec3.UP.scale(60.0)),
                FlightPhase.DESCENT);
        FlightDirector director = new FlightDirector(k, env, body,
                FlightDirector.Mission.BALLISTIC, new Integrator(k), seed);
        return Sim.fly(body, env, director, k, 20000, 1, null, null).hash();
    }

    // ---- I9 ---------------------------------------------------------------

    @SuppressWarnings("unchecked")
    private static void i9NoMagicNumbers(Harness h, Constants k) {
        h.suite("I9 — every physics constant is declared, with units and provenance");

        h.throwsWith("a missing constant fails loudly", Constants.ConstantsException.class,
                () -> k.d("atmosphere.no_such_constant"));
        h.throwsWith("so does a path through a non-object", Constants.ConstantsException.class,
                () -> k.d("gravity.g0.nested"));

        // Every leaf constant must carry units and a source note. A number with no unit and no
        // provenance is exactly the magic number I9 exists to forbid.
        String json;
        try (var in = InvariantTests.class.getResourceAsStream("/physics-constants.json")) {
            json = new String(in.readAllBytes(), java.nio.charset.StandardCharsets.UTF_8);
        } catch (Exception e) {
            h.check("read physics-constants.json", false, e.toString());
            h.endSuite();
            return;
        }
        Map<String, Object> root = Json.parseObject(json);
        int[] counts = {0, 0};
        StringBuilder missing = new StringBuilder();
        auditLeaves("", root, counts, missing);

        h.metric("declared constants", counts[0] + " leaf values");
        h.isTrue("every constant has units and a source note", counts[1] == 0,
                counts[1] == 0 ? "all " + counts[0] + " documented"
                        : counts[1] + " undocumented: " + missing);
        h.endSuite();
    }

    @SuppressWarnings("unchecked")
    private static void auditLeaves(String prefix, Map<String, Object> node,
                                    int[] counts, StringBuilder missing) {
        for (Map.Entry<String, Object> e : node.entrySet()) {
            if (e.getKey().startsWith("_")) continue;
            if (!(e.getValue() instanceof Map)) continue;
            Map<String, Object> m = (Map<String, Object>) e.getValue();
            String path = prefix.isEmpty() ? e.getKey() : prefix + "." + e.getKey();
            if (m.get("value") instanceof Double) {
                counts[0]++;
                boolean documented = m.get("units") instanceof String
                        && m.get("source_note") instanceof String s && !s.isBlank();
                if (!documented) {
                    counts[1]++;
                    missing.append(path).append(' ');
                }
            }
            auditLeaves(path, m, counts, missing);
        }
    }

    // ---- I10 --------------------------------------------------------------

    private static void i10SingleDamageDoor(Harness h, Constants k) {
        h.suite("I10 — kinetics emits events and cannot apply damage");

        Class<?>[] permitted = KineticEvent.class.getPermittedSubclasses();
        h.isTrue("the event hierarchy is sealed",
                KineticEvent.class.isSealed() && permitted != null && permitted.length > 0,
                (permitted == null ? 0 : permitted.length) + " permitted event types, "
                        + "so a consumer cannot invent one");

        // Reflect over every event's every field. None may name or carry damage.
        String[] forbidden = {"damage", "health", "harm", "hurt", "hp", "kill", "destroy"};
        StringBuilder offenders = new StringBuilder();
        int fields = 0;
        for (Class<?> type : permitted == null ? new Class<?>[0] : permitted) {
            for (RecordComponent c : type.getRecordComponents()) {
                fields++;
                String name = c.getName().toLowerCase(java.util.Locale.ROOT);
                for (String bad : forbidden) {
                    if (name.contains(bad)) {
                        offenders.append(type.getSimpleName()).append('.')
                                 .append(c.getName()).append(' ');
                    }
                }
            }
        }
        h.isTrue("no event carries a damage field",
                offenders.isEmpty(),
                offenders.isEmpty()
                        ? "checked " + fields + " fields across "
                          + (permitted == null ? 0 : permitted.length) + " event types"
                        : "offenders: " + offenders);

        // The Impact event exposes kinetic energy, which is what a consumer scales its own
        // effect from - the correct shape for the single door.
        var impact = new KineticEvent.Impact("t", 1.0, Vec3.ZERO, new Vec3(100, 0, 0),
                10.0, "block");
        h.near("Impact reports kinetic energy for the consumer to resolve",
                impact.kineticEnergy(), 0.5 * 10.0 * 100.0 * 100.0, 1e-12, "J");
        h.endSuite();
    }

    // ---- I11 --------------------------------------------------------------

    private static void i11ScaleAudit(Harness h, Constants k) {
        h.suite("I11 — the scale audit is complete and self-consistent");
        ScaleAudit audit = new ScaleAudit(k);
        var inconsistent = audit.inconsistencies();
        h.isTrue("every declared scale factor reconstructs its real value",
                inconsistent.isEmpty(),
                inconsistent.isEmpty() ? k.scaledConstants().size() + " scaled constants, all consistent"
                        : inconsistent.stream().map(c -> c.path()).toList().toString());

        String report = audit.render();
        h.isTrue("the report renders and names every scaled constant",
                k.scaledConstants().stream().allMatch(c -> report.contains(c.path())),
                report.length() + " bytes covering " + k.scaledConstants().size() + " constants");
        h.endSuite();
    }

    // ---- I12 --------------------------------------------------------------

    private static void i12QuaternionSanity(Harness h, Constants k) {
        h.suite("I12 — orientation stays on the unit sphere");
        double tolerance = k.d("limits.quaternion_norm_tolerance");

        // 200,000 integrations at a high rate, which is where naive quaternion integration
        // drifts off the unit sphere and eventually NaNs.
        Quat q = Quat.IDENTITY;
        Vec3 omega = new Vec3(3.7, -2.1, 5.5);
        double worst = 0.0;
        for (int i = 0; i < 200_000; i++) {
            q = q.integrate(omega, 0.0125);
            worst = Math.max(worst, Math.abs(q.norm() - 1.0));
        }
        h.less("norm deviation after 200,000 integrations", worst, tolerance, "");
        h.isTrue("orientation is still finite", q.isFinite(), q.toString());

        // And through a real flight with hard slewing.
        Profile p = Sim.profile(k, "kinetics:interceptor");
        Environment env = Environment.overworld(k, WorldProbe.empty());
        Integrator integrator = new Integrator(k);
        KineticBody body = Sim.body("quat", p, k,
                new Vec3(0, Sim.y(k, 120.0), 0), new Vec3(400, 0, 0), FlightPhase.MIDCOURSE);
        double dt = k.d("world.tick_seconds");
        double flightWorst = 0.0;
        for (int tick = 0; tick < 600; tick++) {
            Vec3 spin = new Vec3(Math.cos(tick * 0.3), Math.sin(tick * 0.7), Math.sin(tick * 0.11))
                    .scale(500.0);
            integrator.step(body, env, ControlCommand.accelerate(spin, 0.0), tick * dt, dt,
                    EventSink.discarding());
            flightWorst = Math.max(flightWorst, Math.abs(body.orientation().norm() - 1.0));
        }
        h.less("norm deviation through 600 ticks of violent slewing", flightWorst, tolerance, "");
        h.endSuite();
    }

    // ---- state machine ----------------------------------------------------

    private static void phaseMachine(Harness h, Constants k) {
        h.suite("Section 2 — the phase machine refuses illegal transitions");
        PhaseMachine machine = new PhaseMachine("t", FlightPhase.RAIL);

        h.isTrue("RAIL -> BOOST is legal",
                machine.transition(FlightPhase.BOOST, 0.0, "ignition", EventSink.discarding()),
                "now " + machine.phase());

        h.throwsWith("BOOST -> LANDED is refused",
                PhaseMachine.IllegalPhaseTransition.class,
                () -> machine.transition(FlightPhase.LANDED, 1.0, "invented",
                        EventSink.discarding()));

        h.isTrue("phase still BOOST after the refusal",
                machine.phase() == FlightPhase.BOOST, machine.phase().toString());

        // Every phase's declared successors must themselves be declared phases, and terminal
        // phases must genuinely terminate.
        int edges = 0;
        for (FlightPhase phase : FlightPhase.values()) {
            edges += FlightPhase.LEGAL.getOrDefault(phase, java.util.EnumSet.noneOf(
                    FlightPhase.class)).size();
        }
        h.metric("declared transitions", edges + " edges across "
                + FlightPhase.values().length + " phases");
        h.isTrue("TERMINATED is absorbing",
                FlightPhase.LEGAL.get(FlightPhase.TERMINATED).isEmpty(), "no way out");
        h.isTrue("thrust exists only in BOOST and TERMINAL",
                java.util.Arrays.stream(FlightPhase.values())
                        .filter(FlightPhase::isPowered).toList()
                        .equals(List.of(FlightPhase.BOOST, FlightPhase.TERMINAL)),
                "TERMINAL is powered because a short-burn interceptor arrives before burnout");
        h.endSuite();
    }
}
