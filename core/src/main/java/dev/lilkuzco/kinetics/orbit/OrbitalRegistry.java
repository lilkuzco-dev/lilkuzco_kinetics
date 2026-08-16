package dev.lilkuzco.kinetics.orbit;

import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.event.EventSink;
import dev.lilkuzco.kinetics.event.KineticEvent;
import dev.lilkuzco.kinetics.math.Vec3;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The orbital registry: satellites that exist without being entities (RE1-RE7).
 *
 * <p>Nothing here is ticked. Every query recomputes from the epoch elements, so an orbit
 * advances at the same rate whether its chunk is loaded, whether the server is lagging, and
 * whether anyone has looked at it in a week (RE5). Two servers running the same world produce
 * identical answers because there is no accumulated state to diverge.
 *
 * <p>Decay (RE4) is the one place where the state depends on history, and it is handled the
 * same way: {@link #semiMajorAxisAt} integrates the decay from the epoch on every call rather
 * than storing a running value. It costs one loop iteration per elapsed orbit, which for a
 * decaying orbit is a few dozen at most before it comes down.
 *
 * <p>Satellites are held in a {@link LinkedHashMap} - insertion-ordered iteration is a
 * determinism requirement (I7), not a preference.
 */
public final class OrbitalRegistry {

    private static final int MAX_DECAY_STEPS = 100_000;

    private final Constants k;
    private final OrbitalMechanics mechanics;
    private final double deorbitAltitude;
    private final double minSustainable;
    private final double decayRateAtFloor;
    private final double decayDepthScale;
    private final double deltaVBudget;
    private final double defaultHalfAngle;

    private final Map<String, Entry> satellites = new LinkedHashMap<>();

    public OrbitalRegistry(Constants k) {
        this.k = k;
        this.mechanics = new OrbitalMechanics(k);
        this.deorbitAltitude = k.d("orbit.deorbit_altitude");
        this.minSustainable = k.d("orbit.minimum_sustainable_altitude");
        this.decayRateAtFloor = k.d("orbit.decay_rate_at_floor");
        this.decayDepthScale = k.d("orbit.decay_depth_scale");
        this.deltaVBudget = k.d("orbit.delta_v_to_orbit");
        this.defaultHalfAngle = k.d("orbit.insertion_reference_footprint_half_angle");
    }

    private record Entry(Orbit orbit, Attitude attitude) {}

    /** A satellite's full state at an instant. */
    public record OrbitalState(
            String id,
            double worldTime,
            Vec3 inertialPosition,
            Vec3 inertialVelocity,
            double semiMajorAxis,
            double altitude,
            double speed,
            double periodSeconds,
            GroundTrack groundTrack,
            boolean decaying) {}

    /** One predicted overflight of a ground point. */
    public record Pass(
            String satelliteId,
            double entryTime,
            double exitTime,
            double closestApproachTime,
            double closestGroundDistance,
            double maxElevationDeg) {

        public double durationSeconds() { return exitTime - entryTime; }
    }

    // ---- registration -----------------------------------------------------

    public void register(Orbit orbit, Attitude attitude) {
        satellites.put(orbit.id(), new Entry(orbit, attitude));
    }

    public boolean remove(String id) { return satellites.remove(id) != null; }

    public boolean contains(String id) { return satellites.containsKey(id); }

    public List<String> ids() { return List.copyOf(satellites.keySet()); }

    public int size() { return satellites.size(); }

    public Orbit orbitOf(String id) {
        Entry e = satellites.get(id);
        return e == null ? null : e.orbit();
    }

    public Attitude attitudeOf(String id) {
        Entry e = satellites.get(id);
        return e == null ? null : e.attitude();
    }

    public OrbitalMechanics mechanics() { return mechanics; }

    // ---- propagation (RE5) ------------------------------------------------

    /**
     * Semi-major axis at a world time, with decay integrated from the epoch (RE4/RE5).
     *
     * <p>An orbit above the sustainable floor never decays, so the answer is the epoch value and
     * the loop is not entered at all. Below the floor the decay rate climbs exponentially with
     * depth, which reproduces the real cliff: a small difference in insertion altitude is the
     * difference between an orbit that lasts and one that is coming down within the hour.
     */
    public double semiMajorAxisAt(Orbit orbit, double worldTime) {
        double a = orbit.semiMajorAxisAtEpoch();
        double elapsed = worldTime - orbit.epochSeconds();
        if (elapsed <= 0.0) return a;

        double floorRadius = mechanics.radiusForAltitude(deorbitAltitude);
        double simulated = 0.0;
        for (int i = 0; i < MAX_DECAY_STEPS && simulated < elapsed; i++) {
            double altitude = mechanics.altitudeForRadius(a);
            if (altitude <= deorbitAltitude) return floorRadius;
            double rate = decayPerOrbit(altitude);
            if (rate <= 0.0) return a;      // stable: nothing further can change
            double period = mechanics.period(a);
            if (period <= 0.0) return a;
            double step = Math.min(period, elapsed - simulated);
            a = Math.max(floorRadius, a - rate * (step / period));
            simulated += step;
        }
        return a;
    }

    /** Semi-major-axis loss per revolution at an altitude, metres (RE4). */
    public double decayPerOrbit(double altitude) {
        if (altitude >= minSustainable) return 0.0;
        double depth = minSustainable - altitude;
        return decayRateAtFloor * Math.exp(depth / decayDepthScale);
    }

    /** Full state at a world time. Null if the satellite is not registered. */
    public OrbitalState stateAt(String id, double worldTime) {
        Entry entry = satellites.get(id);
        if (entry == null) return null;
        return stateOf(entry.orbit(), worldTime);
    }

    public OrbitalState stateOf(Orbit orbit, double worldTime) {
        double a = semiMajorAxisAt(orbit, worldTime);
        double n = mechanics.meanMotion(a);
        double u = Math.toRadians(orbit.argumentOfLatitudeAtEpochDeg())
                + n * (worldTime - orbit.epochSeconds());
        double inc = Math.toRadians(orbit.inclinationDeg());
        double raan = Math.toRadians(orbit.raanDeg());

        double cosU = Math.cos(u);
        double sinU = Math.sin(u);
        double cosI = Math.cos(inc);
        double sinI = Math.sin(inc);
        double cosR = Math.cos(raan);
        double sinR = Math.sin(raan);

        // Unit position in the inertial frame; z is the polar axis. This is the orbital frame,
        // not the Minecraft frame - the conversion to world coordinates happens via lat/lon so
        // that the planet's rotation can be applied in between (RE5b).
        double ux = cosR * cosU - sinR * sinU * cosI;
        double uy = sinR * cosU + cosR * sinU * cosI;
        double uz = sinU * sinI;

        // Derivative with respect to the argument of latitude, times a*n, gives velocity.
        double dx = -cosR * sinU - sinR * cosU * cosI;
        double dy = -sinR * sinU + cosR * cosU * cosI;
        double dz = cosU * sinI;

        Vec3 position = new Vec3(ux, uy, uz).scale(a);
        Vec3 velocity = new Vec3(dx, dy, dz).scale(a * n);

        double latitude = Math.toDegrees(Math.asin(Math.max(-1.0, Math.min(1.0, uz))));
        double inertialLongitude = Math.toDegrees(Math.atan2(uy, ux));
        // RE5b: subtract the planet's rotation. Without this line successive pass predictions
        // are wrong by the whole angle the planet turned through.
        double fixedLongitude = GroundTrack.normalizeLongitude(
                inertialLongitude - mechanics.planetRotationRate() * worldTime);

        double altitude = mechanics.altitudeForRadius(a);
        GroundTrack track = GroundTrack.of(latitude, fixedLongitude, altitude,
                mechanics.planetRadius());

        return new OrbitalState(orbit.id(), worldTime, position, velocity, a, altitude,
                velocity.length(), mechanics.period(a), track, decayPerOrbit(altitude) > 0.0);
    }

    // ---- insertion (RD3 -> RE1) -------------------------------------------

    /** The verdict on a launch reaching the registry. */
    public record InsertionResult(boolean inserted, Orbit orbit, double achievedDeltaV,
                                  double requiredDeltaV, double altitude, String detail) {}

    /**
     * Convert a completed launch into an orbit, or refuse it (RD3).
     *
     * <p>Short of the budget there is no insertion and no partial credit - the vehicle is a
     * ballistic object that happens to be very high, and it comes back down. That is the
     * failure RD3 asks for: underbuilt rockets fail honestly rather than being nudged into
     * orbit.
     *
     * <p>Surplus delta-v above the budget buys altitude, through energy rather than a lookup:
     * the surplus is added to the circular velocity at the decay floor, the resulting specific
     * energy gives a semi-major axis, and the vehicle is circularised there. The response is
     * steep - on a planet this small a hundred extra m/s lifts the orbit by tens of kilometres -
     * which is exactly the leverage a small body gives you.
     */
    public InsertionResult attemptInsertion(String id, double achievedDeltaV, double worldTime,
                                            double inclinationDeg, double raanDeg,
                                            double argumentOfLatitudeDeg, EventSink events) {
        if (achievedDeltaV < deltaVBudget) {
            events.accept(new KineticEvent.InsertionFailed(id, 0.0, achievedDeltaV, deltaVBudget));
            return new InsertionResult(false, null, achievedDeltaV, deltaVBudget, 0.0,
                    String.format("insertion failed: %.1f m/s achieved against a %.1f m/s budget, "
                            + "%.1f m/s short. The vehicle is ballistic and will fall back.",
                            achievedDeltaV, deltaVBudget, deltaVBudget - achievedDeltaV));
        }

        double floorRadius = mechanics.radiusForAltitude(minSustainable);
        double vFloor = mechanics.circularVelocity(floorRadius);
        double surplus = achievedDeltaV - deltaVBudget;
        double v = vFloor + surplus;

        double energy = 0.5 * v * v - mechanics.mu() / floorRadius;
        double a;
        String note;
        if (energy >= 0.0) {
            // Enough surplus to escape. Nothing in v0.1 models a hyperbolic trajectory, so it is
            // capped at a very high bound orbit and said out loud rather than silently clamped.
            a = mechanics.radiusForAltitude(mechanics.referenceAltitude() * 20.0);
            note = String.format("surplus of %.1f m/s exceeds escape velocity; capped at a "
                    + "%.0f m orbit (hyperbolic trajectories are v0.2)", surplus,
                    mechanics.altitudeForRadius(a));
        } else {
            a = -mechanics.mu() / (2.0 * energy);
            note = String.format("inserted at %.0f m with %.1f m/s of surplus delta-v",
                    mechanics.altitudeForRadius(a), surplus);
        }

        Orbit orbit = new Orbit(id, worldTime, a, 0.0, inclinationDeg, raanDeg,
                argumentOfLatitudeDeg);
        register(orbit, Attitude.threeAxis(dev.lilkuzco.kinetics.math.Quat.IDENTITY,
                k.d("limits.max_slew_rate_default")));

        double altitude = mechanics.altitudeForRadius(a);
        events.accept(new KineticEvent.OrbitInsertion(id, 0.0, a, mechanics.period(a),
                achievedDeltaV));
        if (decayPerOrbit(altitude) > 0.0) {
            events.accept(new KineticEvent.OrbitDecaying(id, 0.0, altitude,
                    decayPerOrbit(altitude)));
        }
        return new InsertionResult(true, orbit, achievedDeltaV, deltaVBudget, altitude, note);
    }

    // ---- passes (RE5b) ----------------------------------------------------

    /**
     * Predict the next overflights of a ground point.
     *
     * <p>Sampled rather than solved. An analytic solution exists for a circular orbit over a
     * rotating sphere, but it is fiddly and this runs on demand from a console block, not every
     * tick. The sample step is derived from the footprint and the ground speed so a pass cannot
     * be stepped over: at the reference orbit the footprint is 2887 m and the ground track moves
     * at 1819 m/s, giving a 3.2 s window that is sampled about ten times.
     *
     * @param sensorHalfAngleDeg sensor cone half-angle; 0 or less uses the configured default
     * @param horizonSeconds     how far ahead to look
     */
    public List<Pass> predictPasses(String id, double fromTime, double targetWorldX,
                                    double targetWorldZ, double sensorHalfAngleDeg,
                                    int maxPasses, double horizonSeconds) {
        Entry entry = satellites.get(id);
        if (entry == null) return List.of();
        double halfAngle = sensorHalfAngleDeg > 0.0 ? sensorHalfAngleDeg : defaultHalfAngle;

        OrbitalState probe = stateOf(entry.orbit(), fromTime);
        double footprint = probe.groundTrack().footprintRadius(halfAngle);
        double groundSpeed = probe.speed() * mechanics.planetRadius() / probe.semiMajorAxis();
        double step = groundSpeed > 0.0
                ? Math.max(0.05, footprint / (groundSpeed * 10.0))
                : 1.0;

        List<Pass> passes = new ArrayList<>();
        boolean inside = false;
        double entryTime = 0.0;
        double closestTime = 0.0;
        double closestDistance = Double.MAX_VALUE;

        int steps = (int) Math.ceil(horizonSeconds / step);
        for (int i = 0; i <= steps && passes.size() < maxPasses; i++) {
            double t = fromTime + i * step;
            OrbitalState state = stateOf(entry.orbit(), t);
            double distance = state.groundTrack().groundDistanceTo(
                    targetWorldX, targetWorldZ, mechanics.planetRadius());
            double radius = state.groundTrack().footprintRadius(halfAngle);

            if (distance <= radius) {
                if (!inside) {
                    inside = true;
                    entryTime = t;
                    closestDistance = Double.MAX_VALUE;
                }
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestTime = t;
                }
            } else if (inside) {
                inside = false;
                passes.add(new Pass(id, entryTime, t, closestTime, closestDistance,
                        elevationAngle(closestDistance, probe.altitude())));
            }
        }
        if (inside && passes.size() < maxPasses) {
            double end = fromTime + horizonSeconds;
            passes.add(new Pass(id, entryTime, end, closestTime, closestDistance,
                    elevationAngle(closestDistance, probe.altitude())));
        }
        return passes;
    }

    /** Elevation of the satellite above the horizon at closest approach, degrees. */
    private static double elevationAngle(double groundDistance, double altitude) {
        if (groundDistance <= 0.0) return 90.0;
        return Math.toDegrees(Math.atan2(altitude, groundDistance));
    }

    // ---- deorbit (RE4 -> DESCENT) -----------------------------------------

    /** The handoff from the registry back into the world. */
    public record DeorbitHandoff(String id, Vec3 worldPosition, Vec3 worldVelocity,
                                 double altitude, boolean commanded) {}

    /**
     * Take a satellite out of the registry and hand it to the in-world simulator as a
     * descending body.
     *
     * <p>The velocity handed over is the real orbital velocity, and the vehicle really does
     * have to shed all of it against the air. That is a demanding entry - the atmosphere here
     * carries 1/155th of Earth's column mass because its scale height is compressed, so a
     * reentry vehicle needs a genuinely large drag area for its mass. The library states that
     * rather than quietly softening the entry: a heat shield has to be big, and the physics
     * says how big.
     */
    public DeorbitHandoff deorbit(String id, double worldTime, boolean commanded,
                                  EventSink events) {
        Entry entry = satellites.get(id);
        if (entry == null) return null;
        OrbitalState state = stateOf(entry.orbit(), worldTime);
        GroundTrack track = state.groundTrack();

        // Enter over the current sub-satellite point, at the Karman line, flying along the
        // ground track at orbital speed.
        Vec3 position = new Vec3(track.worldX(), deorbitAltitude, track.worldZ());
        Vec3 heading = groundHeadingOf(entry.orbit(), worldTime);
        Vec3 velocity = heading.scale(state.speed());

        satellites.remove(id);
        events.accept(new KineticEvent.Deorbit(id, 0.0, position, velocity, commanded));
        return new DeorbitHandoff(id, position, velocity, deorbitAltitude, commanded);
    }

    /** Unit direction the ground track is moving, in world coordinates. */
    private Vec3 groundHeadingOf(Orbit orbit, double worldTime) {
        double dt = 0.5;
        GroundTrack now = stateOf(orbit, worldTime).groundTrack();
        GroundTrack next = stateOf(orbit, worldTime + dt).groundTrack();
        double dx = next.worldX() - now.worldX();
        double dz = next.worldZ() - now.worldZ();
        // Guard the longitude wrap: a track crossing +/-180 would otherwise register a jump of
        // most of the planet's circumference in half a second.
        double circumference = 2.0 * Math.PI * mechanics.planetRadius();
        if (dx > circumference * 0.5) dx -= circumference;
        if (dx < -circumference * 0.5) dx += circumference;
        Vec3 heading = new Vec3(dx, 0.0, dz);
        return heading.lengthSq() < 1e-12 ? new Vec3(1, 0, 0) : heading.normalized();
    }

    /**
     * Sweep every registered satellite and deorbit any that have decayed to the floor.
     * Called by consumers on whatever cadence suits them; the result does not depend on how
     * often it runs, because decay is computed from epoch.
     */
    public List<DeorbitHandoff> advanceDecay(double worldTime, EventSink events) {
        List<DeorbitHandoff> handoffs = new ArrayList<>();
        for (String id : List.copyOf(satellites.keySet())) {
            Entry entry = satellites.get(id);
            if (entry == null) continue;
            double a = semiMajorAxisAt(entry.orbit(), worldTime);
            if (mechanics.altitudeForRadius(a) <= deorbitAltitude) {
                DeorbitHandoff handoff = deorbit(id, worldTime, false, events);
                if (handoff != null) handoffs.add(handoff);
            }
        }
        return handoffs;
    }

    /** Advance every satellite's attitude by dt (RE6). Attitude is the one ticked quantity. */
    public void advanceAttitudes(double dt) {
        for (Entry entry : satellites.values()) {
            entry.attitude().advance(dt);
        }
    }
}
