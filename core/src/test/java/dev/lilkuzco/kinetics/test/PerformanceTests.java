package dev.lilkuzco.kinetics.test;

import dev.lilkuzco.kinetics.body.KineticBody;
import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.env.Environment;
import dev.lilkuzco.kinetics.env.WorldProbe;
import dev.lilkuzco.kinetics.event.EventSink;
import dev.lilkuzco.kinetics.guidance.Target;
import dev.lilkuzco.kinetics.integrate.Integrator;
import dev.lilkuzco.kinetics.math.Quat;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.orbit.Attitude;
import dev.lilkuzco.kinetics.orbit.Orbit;
import dev.lilkuzco.kinetics.orbit.OrbitalRegistry;
import dev.lilkuzco.kinetics.phase.FlightDirector;
import dev.lilkuzco.kinetics.phase.FlightPhase;
import dev.lilkuzco.kinetics.profile.Profile;
import dev.lilkuzco.kinetics.util.Rng;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * The performance budget: 500 guided bodies and 50 satellites, twenty minutes, MSPT delta &lt; 2 ms.
 *
 * <p>Twenty minutes means twenty minutes of <em>simulated</em> time - 24,000 ticks - not twenty
 * minutes of waiting. The whole population is stepped every tick exactly as a server would, and
 * what is measured is the wall time of each tick. That is the number that matters: if kinetics
 * takes more than 2 ms of a 50 ms tick, it is eating the server's budget.
 *
 * <p>Bodies that terminate are replaced immediately, so the population stays at 500 for the whole
 * run rather than quietly draining to nothing and flattering the average.
 */
public final class PerformanceTests {

    private static final int GUIDED_BODIES = 500;
    private static final int SATELLITES = 50;
    private static final int SOAK_TICKS = 24_000;      // 20 minutes at 20 Hz
    private static final int WARMUP_TICKS = 2_000;     // let the JIT settle before measuring

    private PerformanceTests() {}

    public static void run(Harness h, Constants k) {
        h.suite("Performance — " + GUIDED_BODIES + " guided bodies + " + SATELLITES
                + " satellites, 20-minute soak");

        Environment env = Environment.overworld(k,
                WorldProbe.flatGround((int) k.d("world.sea_level_y") - 1));
        Integrator integrator = new Integrator(k);
        double dt = k.d("world.tick_seconds");
        EventSink sink = EventSink.discarding();

        List<Flight> flights = new ArrayList<>(GUIDED_BODIES);
        for (int i = 0; i < GUIDED_BODIES; i++) {
            flights.add(spawn(k, env, integrator, i));
        }

        OrbitalRegistry registry = new OrbitalRegistry(k);
        var mechanics = registry.mechanics();
        for (int i = 0; i < SATELLITES; i++) {
            Rng rng = Rng.forPurpose(99, "sat-" + i);
            registry.register(Orbit.circular("sat" + i, mechanics, 0.0,
                    k.d("orbit.reference_orbit_altitude") * rng.range(0.9, 3.0),
                    rng.range(0.0, 98.0), rng.range(0.0, 360.0), rng.range(0.0, 360.0)),
                    i % 2 == 0
                            ? Attitude.threeAxis(Quat.IDENTITY, 5.0)
                            : Attitude.spinStabilized(Quat.IDENTITY, Vec3.UP, 12.0));
        }

        long[] tickNanos = new long[SOAK_TICKS];
        int respawns = 0;
        long registryNanos = 0;

        for (int tick = -WARMUP_TICKS; tick < SOAK_TICKS; tick++) {
            double worldTime = Math.max(tick, 0) * dt;
            long start = System.nanoTime();

            for (int i = 0; i < flights.size(); i++) {
                Flight f = flights.get(i);
                if (!f.body.phase().isInWorld()) {
                    flights.set(i, spawn(k, env, integrator, i + tick * 7919));
                    if (tick >= 0) respawns++;
                    continue;
                }
                f.director.tick(worldTime, dt, f.targetAt(worldTime), null, sink);
            }

            long registryStart = System.nanoTime();
            registry.advanceAttitudes(dt);
            // A server would not query every satellite every tick, but doing so here means the
            // reported number is an upper bound rather than a best case.
            for (String id : registry.ids()) registry.stateAt(id, worldTime);
            long registryEnd = System.nanoTime();

            if (tick >= 0) {
                tickNanos[tick] = registryEnd - start;
                registryNanos += registryEnd - registryStart;
            }
        }

        long[] sorted = tickNanos.clone();
        Arrays.sort(sorted);
        double meanMs = Arrays.stream(tickNanos).average().orElse(0) / 1e6;
        double p50 = sorted[sorted.length / 2] / 1e6;
        double p95 = sorted[(int) (sorted.length * 0.95)] / 1e6;
        double p99 = sorted[(int) (sorted.length * 0.99)] / 1e6;
        double max = sorted[sorted.length - 1] / 1e6;
        double registryMean = registryNanos / (double) SOAK_TICKS / 1e6;

        h.metric("population", GUIDED_BODIES + " in-world bodies, " + SATELLITES
                + " registry satellites, " + respawns + " respawns over the soak");
        h.metric("simulated duration", String.format("%,d ticks = %.1f minutes at 20 Hz",
                SOAK_TICKS, SOAK_TICKS * dt / 60.0));
        h.metric("MSPT mean", String.format("%.4f ms", meanMs));
        h.metric("MSPT p50 / p95 / p99", String.format("%.4f / %.4f / %.4f ms", p50, p95, p99));
        h.metric("MSPT max", String.format("%.4f ms", max));
        h.metric("  of which registry", String.format("%.4f ms (%.1f%%) for %d satellites",
                registryMean, registryMean / Math.max(meanMs, 1e-9) * 100.0, SATELLITES));
        h.metric("per-body cost", String.format("%.2f us", meanMs * 1000.0 / GUIDED_BODIES));

        h.less("MSPT mean is inside the 2 ms budget", meanMs, 2.0, "ms");
        h.less("MSPT p99 is inside the 2 ms budget", p99, 2.0, "ms");
        h.isTrue("the population held at " + GUIDED_BODIES + " for the whole soak",
                flights.size() == GUIDED_BODIES,
                flights.size() + " bodies at the end");
        h.endSuite();
    }

    private record Flight(KineticBody body, FlightDirector director,
                          Vec3 targetOrigin, Vec3 targetVelocity, double spawnTime) {

        Target targetAt(double worldTime) {
            double t = worldTime - spawnTime;
            return new Target("t", targetOrigin.add(targetVelocity.scale(t)),
                    targetVelocity, Vec3.ZERO, 5.0, false);
        }
    }

    /** A fresh engagement, seeded from the index so the soak is reproducible. */
    private static Flight spawn(Constants k, Environment env, Integrator integrator, int index) {
        Rng rng = Rng.forPurpose(4242, "perf-" + index);
        String[] ids = {"kinetics:interceptor", "kinetics:cheap_missile",
                        "kinetics:lofted_missile", "kinetics:mortar_shell"};
        String id = ids[Math.floorMod(index, ids.length)];
        Profile profile = Sim.profile(k, id);

        Vec3 launch = new Vec3(rng.range(-2000, 2000), Sim.y(k, rng.range(5, 120)),
                rng.range(-2000, 2000));
        Vec3 targetPos = launch.add(new Vec3(rng.range(180, 700), rng.range(-40, 60),
                rng.range(-200, 200)));
        Vec3 targetVel = new Vec3(rng.range(-30, 30), 0, rng.range(-30, 30));

        boolean guided = profile.seeker().isPresent();
        Vec3 aim = targetPos.sub(launch).normalized();
        KineticBody body = Sim.body(id + "#" + index, profile, k, launch,
                aim.scale(rng.range(30, 90)),
                guided ? FlightPhase.BOOST : FlightPhase.DESCENT);

        FlightDirector director = new FlightDirector(k, env, body,
                guided ? FlightDirector.Mission.GUIDED : FlightDirector.Mission.BALLISTIC,
                integrator, index);
        return new Flight(body, director, targetPos, targetVel, 0.0);
    }
}
