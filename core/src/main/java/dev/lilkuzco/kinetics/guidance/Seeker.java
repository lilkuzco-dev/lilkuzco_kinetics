package dev.lilkuzco.kinetics.guidance;

import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.env.WorldProbe;
import dev.lilkuzco.kinetics.event.EventSink;
import dev.lilkuzco.kinetics.event.KineticEvent;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.profile.SeekerSpec;

/**
 * Seeker head with a field of view and a lock state machine (RC6).
 *
 * <p>This class exists to stop seekers being omniscient, and that single restriction is what
 * turns a missile from a homing arrow into something a player can fight. A seeker sees only
 * what is inside its gimbal cone and not hidden behind terrain. Break either condition and it
 * loses lock, coasts toward where it last computed the intercept would be, and reacquires only
 * if the target comes back into view.
 *
 * <p>Two real tactics fall out of that with no further code:
 * <ul>
 *   <li><b>Terrain masking</b> - put a ridge between yourself and the missile and the LOS check
 *       fails. It flies at your last predicted position and passes behind the hill.</li>
 *   <li><b>Notching</b> - turn hard across the seeker's nose and the line of sight sweeps
 *       faster than the head can gimbal. The cone loses you even in clear air.</li>
 * </ul>
 *
 * <p>The lock states are ordered by how much the seeker knows: LOCKED (measuring),
 * MEMORY_TRACK (extrapolating a fading estimate), INERTIAL (flying a straight line and hoping).
 * There is deliberately no path that silently returns to the true target position without
 * passing back through acquisition.
 */
public final class Seeker {

    public enum LockState {
        /** No lock yet; the head is looking. */
        SEARCHING,
        /** Measuring the target directly. Full guidance quality. */
        LOCKED,
        /** Lost sight; flying the last predicted intercept while the memory timer runs (RC6). */
        MEMORY_TRACK,
        /** Memory expired. Ballistic, no guidance. */
        INERTIAL
    }

    private final SeekerSpec spec;
    private final String bodyId;
    private final double horizonStep;

    private LockState state = LockState.SEARCHING;
    private Target tracked;
    private Vec3 memoryAimPoint = Vec3.ZERO;
    private double memoryRemaining;
    private Vec3 previousLos;
    private double lastCrossingRateDeg;

    public Seeker(String bodyId, SeekerSpec spec, Constants k) {
        this.bodyId = bodyId;
        this.spec = spec;
        this.horizonStep = k.d("sensors.horizon_sample_step");
    }

    /** What the seeker believes this tick. */
    public record Fix(LockState state, Target target, Vec3 aimPoint, double crossingRateDeg) {

        /** Whether guidance should steer at all this tick. */
        public boolean guides() { return state == LockState.LOCKED || state == LockState.MEMORY_TRACK; }
    }

    /**
     * Update the lock.
     *
     * @param boresight the seeker's look direction, normally the body's forward axis
     * @param truth     where the target actually is; visible to the seeker only if the cone and
     *                  line-of-sight tests pass
     */
    public Fix update(Vec3 position, Vec3 velocity, Vec3 boresight, Target truth,
                      WorldProbe world, double dt, double bodyAge, EventSink events) {

        if (!spec.isPresent()) {
            return new Fix(LockState.INERTIAL, null, Vec3.ZERO, 0.0);
        }

        boolean visible = truth != null && canSee(position, boresight, truth, world);
        double crossingRate = crossingRate(position, truth, dt);
        boolean trackable = visible && crossingRate <= spec.maxCrossingRateDeg();

        if (trackable) {
            boolean reacquisition = state == LockState.MEMORY_TRACK || state == LockState.INERTIAL;
            if (state != LockState.LOCKED) {
                events.accept(new KineticEvent.LockAcquired(
                        bodyId, bodyAge, truth.id(), reacquisition));
            }
            state = LockState.LOCKED;
            tracked = truth;
            memoryRemaining = spec.memoryTrackSeconds();
            memoryAimPoint = ProportionalNavigation.predictedInterceptPoint(position, velocity, truth);
            return new Fix(state, truth, memoryAimPoint, crossingRate);
        }

        // Not trackable this tick.
        if (state == LockState.LOCKED) {
            String cause = truth == null ? "target gone"
                    : !visible ? (withinCone(position, boresight, truth)
                            ? "line of sight blocked" : "outside gimbal cone")
                    : "crossing rate exceeded";
            events.accept(new KineticEvent.LockLost(
                    bodyId, bodyAge, tracked != null ? tracked.id() : "?", cause,
                    spec.memoryTrackSeconds()));
            state = LockState.MEMORY_TRACK;
            memoryRemaining = spec.memoryTrackSeconds();
        }

        if (state == LockState.MEMORY_TRACK) {
            memoryRemaining -= dt;
            if (memoryRemaining <= 0.0) {
                events.accept(new KineticEvent.LockExpired(
                        bodyId, bodyAge, tracked != null ? tracked.id() : "?"));
                state = LockState.INERTIAL;
                return new Fix(state, null, Vec3.ZERO, crossingRate);
            }
            // Keep flying the aim point computed when the lock was last good. It does not
            // update - that is the whole cost of losing lock.
            return new Fix(state, memoryGhost(), memoryAimPoint, crossingRate);
        }

        return new Fix(state, null, Vec3.ZERO, crossingRate);
    }

    /**
     * A synthetic target at the remembered aim point, so downstream guidance can run unchanged
     * while blind. It carries no velocity: the seeker has no current measurement, and inventing
     * one would be the "silent re-cheat back onto the true target" RF5 forbids.
     */
    private Target memoryGhost() {
        String id = tracked != null ? tracked.id() : "memory";
        return new Target(id, memoryAimPoint, Vec3.ZERO, Vec3.ZERO,
                tracked != null ? tracked.signature() : 1.0, false);
    }

    private boolean canSee(Vec3 position, Vec3 boresight, Target target, WorldProbe world) {
        if (!withinCone(position, boresight, target)) return false;
        double range = target.position().sub(position).length();
        if (!spec.withinEnvelope(range, target.position().y())) return false;
        return world == null || world.lineOfSight(position, target.position(), horizonStep);
    }

    private boolean withinCone(Vec3 position, Vec3 boresight, Target target) {
        Vec3 los = target.position().sub(position);
        if (los.lengthSq() < 1e-12 || boresight.lengthSq() < 1e-12) return true;
        double offAxisDeg = Math.toDegrees(boresight.angleTo(los));
        return offAxisDeg <= spec.fieldOfViewDeg();
    }

    /**
     * How fast the line of sight is sweeping, deg/s. A target that crosses faster than the head
     * can follow is lost even in clear air - this is notching.
     */
    private double crossingRate(Vec3 position, Target target, double dt) {
        if (target == null || dt <= 0.0) { previousLos = null; return 0.0; }
        Vec3 los = target.position().sub(position).normalized();
        if (previousLos == null || los.lengthSq() == 0.0) {
            previousLos = los;
            return 0.0;
        }
        double rate = Math.toDegrees(previousLos.angleTo(los)) / dt;
        previousLos = los;
        lastCrossingRateDeg = rate;
        return rate;
    }

    /** Force the seeker onto a decoy (RF5). Never silently reverts to the true target. */
    public void seduceOnto(Target decoy, String countermeasure, double bodyAge, EventSink events) {
        String from = tracked != null ? tracked.id() : "?";
        events.accept(new KineticEvent.DecoySeduced(
                bodyId, bodyAge, from, decoy.id(), countermeasure));
        tracked = decoy;
        state = LockState.LOCKED;
        memoryRemaining = spec.memoryTrackSeconds();
    }

    /** Break the lock into memory-track without a decoy (chaff burst, jamming). */
    public void breakLock(String cause, double bodyAge, EventSink events) {
        if (state != LockState.LOCKED) return;
        events.accept(new KineticEvent.LockLost(
                bodyId, bodyAge, tracked != null ? tracked.id() : "?", cause,
                spec.memoryTrackSeconds()));
        state = LockState.MEMORY_TRACK;
        memoryRemaining = spec.memoryTrackSeconds();
    }

    public LockState state() { return state; }

    public Target tracked() { return tracked; }

    public double memoryRemaining() { return memoryRemaining; }

    public double lastCrossingRateDeg() { return lastCrossingRateDeg; }

    public SeekerSpec spec() { return spec; }
}
