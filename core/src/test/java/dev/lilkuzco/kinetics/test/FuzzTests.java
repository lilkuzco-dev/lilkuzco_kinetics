package dev.lilkuzco.kinetics.test;

import dev.lilkuzco.kinetics.body.KineticBody;
import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.env.Atmosphere;
import dev.lilkuzco.kinetics.env.Environment;
import dev.lilkuzco.kinetics.env.WindField;
import dev.lilkuzco.kinetics.env.WorldProbe;
import dev.lilkuzco.kinetics.event.EventSink;
import dev.lilkuzco.kinetics.guidance.Target;
import dev.lilkuzco.kinetics.integrate.ControlCommand;
import dev.lilkuzco.kinetics.integrate.Integrator;
import dev.lilkuzco.kinetics.invariant.InvariantViolation;
import dev.lilkuzco.kinetics.invariant.Invariants;
import dev.lilkuzco.kinetics.math.Quat;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.phase.FlightDirector;
import dev.lilkuzco.kinetics.phase.FlightPhase;
import dev.lilkuzco.kinetics.profile.Profile;
import dev.lilkuzco.kinetics.profile.ProfileLoader;
import dev.lilkuzco.kinetics.util.Rng;

import java.util.ArrayList;
import java.util.List;

/**
 * Invariant I8: 10,000 randomised profiles and scenarios per CI run, zero breaches.
 *
 * <p>Two halves, doing different jobs.
 *
 * <p><b>Absurd profiles must be rejected at load.</b> Negative mass, zero area, a vacuum Isp
 * below the sea-level figure - these are caught by {@link ProfileLoader} with an explanation,
 * because the alternative is a NaN appearing in the integrator three seconds later where the
 * cause is unrecoverable. This half asserts the rejection actually happens rather than trusting
 * that it does.
 *
 * <p><b>Valid-but-extreme profiles must fly without breaching anything.</b> Thousand-tonne
 * bodies, gram-scale bodies, absurd drag areas, launches straight into the ground, guidance
 * demanding hundreds of g, wind switched on, vacuum and 1 g dimensions - all of it, and not one
 * NaN, teleport, energy gain or quaternion drift.
 *
 * <p>Everything is seeded, so a failure is reproducible from the seed printed with it.
 */
public final class FuzzTests {

    private static final int PROFILE_CASES = 4_000;
    private static final int FLIGHT_CASES = 6_000;

    private FuzzTests() {}

    public static void run(Harness h, Constants k) {
        rejectAbsurdProfiles(h, k);
        flyExtremeProfiles(h, k);
    }

    // ---- half one: rejection ----------------------------------------------

    private static void rejectAbsurdProfiles(Harness h, Constants k) {
        h.suite("I8 — " + PROFILE_CASES + " absurd profiles, all rejected at load");
        ProfileLoader loader = new ProfileLoader(k);

        int accepted = 0;
        int rejected = 0;
        List<String> escaped = new ArrayList<>();

        for (int i = 0; i < PROFILE_CASES; i++) {
            Rng rng = Rng.forPurpose(0xBADCAFE, "absurd-" + i);
            String json = absurdProfile(rng, i);
            try {
                loader.load(json);
                accepted++;
                if (escaped.size() < 5) escaped.add("case " + i + ": " + json);
            } catch (ProfileLoader.ProfileException | dev.lilkuzco.kinetics.util.Json.JsonException e) {
                rejected++;
            }
        }
        h.isTrue("every malformed profile was rejected with an explanation",
                accepted == 0,
                rejected + " rejected, " + accepted + " slipped through"
                        + (escaped.isEmpty() ? "" : " — e.g. " + escaped.get(0)));

        // And the specific rejections I8 names, each checked by hand so the messages stay useful.
        h.throwsWith("negative mass", ProfileLoader.ProfileException.class,
                () -> new ProfileLoader(k).load("""
                        {"id":"t","payload_dry_mass":-5.0,
                         "airframe":{"reference_area":1.0,"cd0":0.3}}"""));
        h.throwsWith("zero reference area", ProfileLoader.ProfileException.class,
                () -> new ProfileLoader(k).load("""
                        {"id":"t","payload_dry_mass":10.0,
                         "airframe":{"reference_area":0.0,"cd0":0.3}}"""));
        h.throwsWith("vacuum Isp below sea-level Isp", ProfileLoader.ProfileException.class,
                () -> new ProfileLoader(k).load("""
                        {"id":"t","payload_dry_mass":10.0,
                         "airframe":{"reference_area":1.0,"cd0":0.3},
                         "stages":[{"fuel_mass":10.0,"stage_dry_mass":1.0,"thrust_vacuum":100.0,
                                    "isp_sea_level":320.0,"isp_vacuum":280.0}]}"""));
        h.throwsWith("negative drag coefficient", ProfileLoader.ProfileException.class,
                () -> new ProfileLoader(k).load("""
                        {"id":"t","payload_dry_mass":10.0,
                         "airframe":{"reference_area":1.0,"cd0":-0.5}}"""));
        h.throwsWith("a main chute listed above its drogue", ProfileLoader.ProfileException.class,
                () -> new ProfileLoader(k).load("""
                        {"id":"t","payload_dry_mass":10.0,
                         "airframe":{"reference_area":1.0,"cd0":0.3},
                         "recovery":[{"name":"drogue","cd":1.4,"area":2.0,"deploy_altitude":50.0},
                                     {"name":"main","cd":1.6,"area":9.0,"deploy_altitude":150.0}]}"""));
        h.throwsWith("four stages", ProfileLoader.ProfileException.class,
                () -> new ProfileLoader(k).load("""
                        {"id":"t","payload_dry_mass":10.0,
                         "airframe":{"reference_area":1.0,"cd0":0.3},
                         "stages":[{"fuel_mass":1.0,"stage_dry_mass":1.0,"thrust_vacuum":10.0,
                                    "isp_sea_level":200.0,"isp_vacuum":220.0},
                                   {"fuel_mass":1.0,"stage_dry_mass":1.0,"thrust_vacuum":10.0,
                                    "isp_sea_level":200.0,"isp_vacuum":220.0},
                                   {"fuel_mass":1.0,"stage_dry_mass":1.0,"thrust_vacuum":10.0,
                                    "isp_sea_level":200.0,"isp_vacuum":220.0},
                                   {"fuel_mass":1.0,"stage_dry_mass":1.0,"thrust_vacuum":10.0,
                                    "isp_sea_level":200.0,"isp_vacuum":220.0}]}"""));
        h.endSuite();
    }

    /** A profile with one or more fields deliberately poisoned. */
    private static String absurdProfile(Rng rng, int index) {
        double mass = switch (index % 6) {
            case 0 -> -rng.range(1, 1e6);
            case 1 -> 0.0;
            case 2 -> Double.NaN;
            case 3 -> Double.POSITIVE_INFINITY;
            default -> rng.range(1, 1000);
        };
        double area = switch (index % 5) {
            case 0 -> 0.0;
            case 1 -> -rng.range(0.1, 100);
            case 2 -> Double.NaN;
            default -> rng.range(0.01, 50);
        };
        double cd = switch (index % 4) {
            case 0 -> -rng.range(0.1, 5);
            case 1 -> Double.NaN;
            default -> rng.range(0.0, 3);
        };
        double ispSl = rng.range(1, 500);
        double ispVac = switch (index % 3) {
            case 0 -> ispSl - rng.range(1, 200);   // backwards
            case 1 -> -rng.range(1, 400);
            default -> ispSl + rng.range(1, 100);
        };
        // At least one of the four is poisoned for every index, because the modulos are
        // coprime enough that no index escapes all of them.
        boolean allValid = mass > 0 && Double.isFinite(mass)
                && area > 0 && Double.isFinite(area)
                && cd >= 0 && Double.isFinite(cd)
                && ispVac >= ispSl;
        if (allValid) area = 0.0;   // force at least one fault

        return String.format(java.util.Locale.ROOT, """
                {"id":"fuzz:%d","payload_dry_mass":%s,
                 "airframe":{"reference_area":%s,"cd0":%s},
                 "stages":[{"fuel_mass":%.3f,"stage_dry_mass":%.3f,"thrust_vacuum":%.3f,
                            "isp_sea_level":%.3f,"isp_vacuum":%.3f}]}""",
                index, json(mass), json(area), json(cd),
                rng.range(0, 5000), rng.range(1, 500), rng.range(1, 1e6), ispSl, ispVac);
    }

    /** NaN and Infinity are not valid JSON numbers; emit them as tokens the parser rejects. */
    private static String json(double v) {
        if (Double.isNaN(v)) return "NaN";
        if (Double.isInfinite(v)) return v > 0 ? "Infinity" : "-Infinity";
        return String.format(java.util.Locale.ROOT, "%.6f", v);
    }

    // ---- half two: extreme but valid --------------------------------------

    private static void flyExtremeProfiles(Harness h, Constants k) {
        h.suite("I8 — " + FLIGHT_CASES + " extreme scenarios, zero invariant breaches");

        int flown = 0;
        int breaches = 0;
        String firstBreach = null;
        long firstBreachSeed = 0;
        double worstSpeed = 0.0;
        int totalTicks = 0;

        long start = System.nanoTime();
        for (int i = 0; i < FLIGHT_CASES; i++) {
            long seed = 0x5EEDL + i;
            Rng rng = new Rng(seed);
            try {
                totalTicks += flyOne(k, rng, seed);
                flown++;
            } catch (InvariantViolation v) {
                breaches++;
                if (firstBreach == null) {
                    firstBreach = v.getMessage();
                    firstBreachSeed = seed;
                }
            }
        }
        long ms = (System.nanoTime() - start) / 1_000_000;

        h.isTrue("no invariant breached across " + FLIGHT_CASES + " randomised flights",
                breaches == 0,
                breaches == 0
                        ? String.format("%d flights, %d ticks integrated, %d ms", flown,
                                totalTicks, ms)
                        : String.format("%d breaches; first at seed %d:%n%s", breaches,
                                firstBreachSeed, firstBreach));
        h.metric("throughput",
                String.format("%,d body-ticks in %d ms (%.0f k ticks/s)",
                        totalTicks, ms, totalTicks / Math.max(ms, 1) / 1.0));
        h.endSuite();
    }

    /** One randomised flight. Returns the number of ticks integrated. */
    private static int flyOne(Constants k, Rng rng, long seed) {
        double mass = Math.pow(10.0, rng.range(-2.0, 5.0));       // 10 mg to 100 tonnes
        double area = Math.pow(10.0, rng.range(-3.0, 2.0));       // 1 mm^2 to 100 m^2
        double cd = rng.range(0.01, 2.5);
        double wingArea = rng.chance(0.5) ? 0.0 : area * rng.range(0.1, 20.0);
        double gLimit = rng.range(1.0, 80.0);                     // above the cap on purpose

        StringBuilder json = new StringBuilder();
        json.append(String.format(java.util.Locale.ROOT, """
                {"id":"fuzz:flight","payload_dry_mass":%.6f,"max_slew_rate_deg":%.3f,
                 "airframe":{"reference_area":%.6f,"cd0":%.4f,"wing_area":%.6f,
                             "aspect_ratio":%.3f,"g_limit":%.3f,"q_max":%.1f,
                             "nose_radius":%.4f}""",
                mass, rng.range(1.0, 400.0), area, cd, wingArea,
                rng.range(0.5, 20.0), gLimit, rng.range(1000.0, 500000.0),
                rng.range(0.01, 3.0)));

        boolean powered = rng.chance(0.4);
        if (powered) {
            double ispSl = rng.range(50.0, 450.0);
            json.append(String.format(java.util.Locale.ROOT, """
                    ,"stages":[{"fuel_mass":%.4f,"stage_dry_mass":%.4f,"thrust_vacuum":%.4f,
                                "isp_sea_level":%.3f,"isp_vacuum":%.3f}]""",
                    mass * rng.range(0.1, 20.0), mass * rng.range(0.05, 2.0),
                    rng.range(1.0, 2.0e6), ispSl, ispSl + rng.range(0.0, 60.0)));
        }
        if (rng.chance(0.3)) {
            json.append(String.format(java.util.Locale.ROOT, """
                    ,"recovery":[{"name":"c","cd":%.3f,"area":%.4f,"q_deploy_max":%.1f,
                                  "deploy_altitude":%.2f}]""",
                    rng.range(0.5, 2.0), area * rng.range(1.0, 60.0),
                    rng.range(100.0, 60000.0), rng.range(5.0, 240.0)));
        }
        json.append('}');

        Profile profile = new ProfileLoader(k).load(json.toString());

        // Environment: sometimes vacuum, sometimes wind, sometimes solid ground, sometimes not.
        Environment env = new Environment(k,
                rng.chance(0.15) ? Atmosphere.vacuum(k) : Atmosphere.standard(k),
                rng.chance(0.3) ? WindField.seeded(k, seed) : WindField.disabled(k),
                rng.chance(0.6) ? WorldProbe.flatGround((int) k.d("world.sea_level_y") - 1)
                        : WorldProbe.empty(),
                rng.chance(0.2) ? k.d("gravity.dimension_scalars.moon") : 1.0);

        Vec3 position = new Vec3(rng.range(-500, 500),
                Sim.y(k, rng.range(-40.0, 250.0)), rng.range(-500, 500));
        Vec3 velocity = new Vec3(rng.range(-600, 600), rng.range(-600, 600), rng.range(-600, 600));

        KineticBody body = new KineticBody("fuzz", profile, k, position, velocity,
                Quat.fromAxisAngle(new Vec3(rng.range(-1, 1), rng.range(-1, 1), rng.range(-1, 1)),
                        rng.range(0, Math.PI * 2)),
                powered ? FlightPhase.RAIL : FlightPhase.DESCENT);

        Integrator integrator = new Integrator(k, new Invariants(k));
        FlightDirector director = new FlightDirector(k, env, body,
                powered ? FlightDirector.Mission.LAUNCH : FlightDirector.Mission.BALLISTIC,
                integrator, seed);

        double dt = k.d("world.tick_seconds");
        int ticks = rng.nextInt(300) + 20;
        int done = 0;
        for (int t = 0; t < ticks && body.phase().isInWorld(); t++) {
            director.tick(t * dt, dt, null, null, EventSink.discarding());
            done++;
        }

        // A second pass driving the raw integrator with deliberately absurd guidance commands,
        // which the flight director would never produce.
        if (body.phase().isInWorld()) {
            for (int t = 0; t < 60 && body.phase().isInWorld(); t++) {
                Vec3 insane = new Vec3(rng.range(-1e4, 1e4), rng.range(-1e4, 1e4),
                        rng.range(-1e4, 1e4));
                integrator.step(body, env,
                        rng.chance(0.5) ? ControlCommand.accelerate(insane, rng.range(0, 1))
                                : ControlCommand.pointAt(insane, rng.range(0, 1)),
                        t * dt, dt, EventSink.discarding());
                done++;
            }
        }
        return done;
    }
}
