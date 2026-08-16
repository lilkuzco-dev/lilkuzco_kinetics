package dev.lilkuzco.kinetics.test;

import dev.lilkuzco.kinetics.body.BodyState;
import dev.lilkuzco.kinetics.body.KineticBody;
import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.env.Environment;
import dev.lilkuzco.kinetics.event.EventSink;
import dev.lilkuzco.kinetics.guidance.Target;
import dev.lilkuzco.kinetics.integrate.Integrator;
import dev.lilkuzco.kinetics.math.Quat;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.phase.FlightDirector;
import dev.lilkuzco.kinetics.phase.FlightPhase;
import dev.lilkuzco.kinetics.profile.Profile;
import dev.lilkuzco.kinetics.profile.ProfileLoader;
import dev.lilkuzco.kinetics.sensors.Countermeasures;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;

/** Shared plumbing for the battery: profile loading, flight loops and trajectory traces. */
public final class Sim {

    private static Map<String, Profile> cached;

    private Sim() {}

    /** The shipped reference profiles, by id. */
    public static Map<String, Profile> profiles(Constants k) {
        if (cached != null) return cached;
        try (InputStream in = Sim.class.getResourceAsStream("/profiles/kinetics-default.json")) {
            if (in == null) throw new IllegalStateException(
                    "/profiles/kinetics-default.json is not on the classpath");
            String json = new String(in.readAllBytes(), StandardCharsets.UTF_8);
            Map<String, Profile> map = new LinkedHashMap<>();
            for (Profile p : new ProfileLoader(k).loadAll(json)) map.put(p.id(), p);
            cached = map;
            return map;
        } catch (IOException e) {
            throw new IllegalStateException("could not read the default profiles", e);
        }
    }

    public static Profile profile(Constants k, String id) {
        Profile p = profiles(k).get(id);
        if (p == null) throw new IllegalArgumentException("no such profile: " + id
                + " (have " + profiles(k).keySet() + ")");
        return p;
    }

    /** One recorded flight. */
    public static final class Trace {
        public final List<BodyState> states = new ArrayList<>();
        public final EventSink.Recording events = new EventSink.Recording();
        public int ticks;
        public double peakSpeed;
        public double peakAltitude;
        public double peakDynamicPressure;
        public double peakHeating;
        public double minSpeedAfterPeak = Double.MAX_VALUE;
        public FlightPhase finalPhase;
        public boolean landed;

        public BodyState last() { return states.isEmpty() ? null : states.get(states.size() - 1); }

        public BodyState first() { return states.isEmpty() ? null : states.get(0); }

        /** SHA-256 over the exact bits of every recorded state (I7). */
        public String hash() {
            MessageDigest digest = BodyState.newDigest();
            for (BodyState s : states) s.hashInto(digest);
            return BodyState.hex(digest.digest());
        }

        /** First few and last few states, for a failure diff. */
        public String renderEnds(int n) {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < Math.min(n, states.size()); i++) {
                sb.append("    ").append(states.get(i).toLine()).append('\n');
            }
            if (states.size() > 2 * n) sb.append("    ...\n");
            for (int i = Math.max(n, states.size() - n); i < states.size(); i++) {
                sb.append("    ").append(states.get(i).toLine()).append('\n');
            }
            return sb.toString();
        }
    }

    /**
     * Fly a body until it reaches a terminal phase or the tick budget runs out.
     *
     * @param sampleEvery record a state every N ticks; 1 records everything
     */
    public static Trace fly(KineticBody body, Environment env, FlightDirector director,
                            Constants k, int maxTicks, int sampleEvery,
                            Target target, List<Countermeasures.Decoy> decoys) {
        return fly(body, env, director, k, maxTicks, sampleEvery, target, decoys, null);
    }

    public static Trace fly(KineticBody body, Environment env, FlightDirector director,
                            Constants k, int maxTicks, int sampleEvery,
                            Target target, List<Countermeasures.Decoy> decoys,
                            BiConsumer<Integer, KineticBody> perTick) {
        return fly(body, env, director, k, maxTicks, sampleEvery, target, decoys, perTick, null);
    }

    /**
     * Fly until a terminal phase, the tick budget, or {@code stopWhen} - whichever comes first.
     *
     * <p>The stop predicate matters for tests run in an empty vacuum world, where there is no
     * ground to land on and a body would otherwise fall until it tripped the speed ceiling.
     */
    public static Trace fly(KineticBody body, Environment env, FlightDirector director,
                            Constants k, int maxTicks, int sampleEvery,
                            Target target, List<Countermeasures.Decoy> decoys,
                            BiConsumer<Integer, KineticBody> perTick,
                            java.util.function.Predicate<KineticBody> stopWhen) {
        Trace trace = new Trace();
        double dt = k.d("world.tick_seconds");
        boolean peaked = false;

        for (int tick = 0; tick < maxTicks; tick++) {
            if (!body.phase().isInWorld()) break;
            if (stopWhen != null && stopWhen.test(body)) break;
            director.tick(tick * dt, dt, target, decoys, trace.events);
            trace.ticks = tick + 1;

            double speed = body.speed();
            double altitude = env.altitudeOf(body.position().y());
            if (speed > trace.peakSpeed) { trace.peakSpeed = speed; peaked = false; }
            else if (trace.peakSpeed > 0.0) { peaked = true; }
            if (peaked && speed < trace.minSpeedAfterPeak) trace.minSpeedAfterPeak = speed;
            if (altitude > trace.peakAltitude) trace.peakAltitude = altitude;
            if (body.dynamicPressure() > trace.peakDynamicPressure) {
                trace.peakDynamicPressure = body.dynamicPressure();
            }
            if (body.heatingRate() > trace.peakHeating) trace.peakHeating = body.heatingRate();

            if (tick % sampleEvery == 0) trace.states.add(body.snapshot());
            if (perTick != null) perTick.accept(tick, body);
        }
        trace.states.add(body.snapshot());
        trace.finalPhase = body.phase();
        trace.landed = body.phase() == FlightPhase.LANDED;
        return trace;
    }

    /** A body pointed along its velocity, ready to be flown. */
    public static KineticBody body(String id, Profile profile, Constants k,
                                   Vec3 position, Vec3 velocity, FlightPhase phase) {
        Quat facing = velocity.lengthSq() > 1e-12
                ? Quat.between(new Vec3(0, 0, 1), velocity.normalized())
                : Quat.between(new Vec3(0, 0, 1), Vec3.UP);
        return new KineticBody(id, profile, k, position, velocity, facing, phase);
    }

    /** World y for an altitude above the datum. */
    public static double y(Constants k, double altitude) {
        return k.d("world.sea_level_y") + altitude;
    }

    public static Integrator integrator(Constants k) { return new Integrator(k); }

    /** A vacuum environment at 1 g: no drag, no lift, vacuum Isp everywhere. */
    public static Environment vacuum(Constants k) {
        return new Environment(k,
                dev.lilkuzco.kinetics.env.Atmosphere.vacuum(k),
                dev.lilkuzco.kinetics.env.WindField.disabled(k),
                dev.lilkuzco.kinetics.env.WorldProbe.empty(), 1.0);
    }

    /**
     * Constants with the atmospheric scale height blown up so density is effectively uniform.
     *
     * <p>Used only by the terminal-velocity cross-check. Terminal velocity is defined for a
     * <em>fixed</em> density, and in the real exponential atmosphere a falling body chases a
     * target that moves as it descends - it lags by a few percent through no fault of the
     * integrator. Flattening the atmosphere isolates the quantity actually under test. Every
     * other test runs against the shipped constants.
     */
    public static Constants uniformAtmosphere() {
        try (InputStream in = Sim.class.getResourceAsStream("/physics-constants.json")) {
            String json = new String(in.readAllBytes(), StandardCharsets.UTF_8);
            String anchor = "\"scale_height\": {\n      \"value\": 55.0,";
            if (!json.contains(anchor)) {
                // A replace that finds nothing is a silent no-op that would leave this test
                // quietly measuring the wrong thing. Fail loudly instead.
                throw new IllegalStateException(
                        "uniformAtmosphere() anchor no longer matches physics-constants.json; "
                        + "the scale_height block was edited. Fix the anchor.");
            }
            return Constants.fromText(json.replace(anchor,
                    "\"scale_height\": {\n      \"value\": 1.0e9,"));
        } catch (IOException e) {
            throw new IllegalStateException("could not read physics-constants.json", e);
        }
    }
}
