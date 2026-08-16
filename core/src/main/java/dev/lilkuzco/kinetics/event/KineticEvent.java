package dev.lilkuzco.kinetics.event;

import dev.lilkuzco.kinetics.math.Vec3;

/**
 * Everything kinetics is allowed to tell the world. This is the single damage door (I10).
 *
 * <p>Kinetics never applies damage, never breaks blocks, never kills an entity. It reports
 * that a body arrived somewhere at some speed, or that a structural limit was passed, and the
 * consumer decides what that means - warfront resolves it through
 * {@code AreaStrike.resolve()}, cosmos decides what a shredded parachute looks like. Any
 * damage code appearing in this library is rejected at review, and this sealed hierarchy is
 * what makes that reviewable: there is no event here that carries a damage value, because
 * there is nowhere for one to go.
 *
 * <p>Sealed deliberately. A consumer cannot invent a new event type, and adding one here is a
 * visible API change rather than a quiet extension.
 */
public sealed interface KineticEvent {

    /** Identifier of the body that produced this event. */
    String bodyId();

    /** Seconds since that body was created. */
    double bodyAge();

    // ---- flight ----------------------------------------------------------

    /** A body struck terrain or a target. Carries kinematics; carries no damage. */
    record Impact(String bodyId, double bodyAge, Vec3 position, Vec3 velocity,
                  double mass, String struck) implements KineticEvent {

        /** Kinetic energy at impact, J. What a consumer scales its own effect from. */
        public double kineticEnergy() { return 0.5 * mass * velocity.lengthSq(); }
    }

    /** A proximity fuse condition was met - closest approach inside the fuse radius. */
    record Proximity(String bodyId, double bodyAge, Vec3 position, Vec3 velocity,
                     double mass, String targetId, double missDistance) implements KineticEvent {

        public double kineticEnergy() { return 0.5 * mass * velocity.lengthSq(); }
    }

    /** Dynamic pressure exceeded the airframe's q_max (RB6). */
    record StructuralLimit(String bodyId, double bodyAge, Vec3 position,
                           double dynamicPressure, double qMax) implements KineticEvent {}

    /** A parachute was opened above its deploy limit and shredded (RB6). */
    record ChuteShred(String bodyId, double bodyAge, String chuteName,
                      double dynamicPressure, double qDeployMax) implements KineticEvent {}

    /** A parachute opened successfully. */
    record ChuteDeployed(String bodyId, double bodyAge, String chuteName,
                         double altitude, double dynamicPressure) implements KineticEvent {}

    /** Reentry heating passed the profile threshold (RE7). Reported, never applied. */
    record ReentryOverheat(String bodyId, double bodyAge, double heatingRate,
                           double threshold, double altitude) implements KineticEvent {}

    /** A stage burned out and was shed (RD4). */
    record Staging(String bodyId, double bodyAge, int stageIndex, double shedMass,
                   double velocityAtStaging) implements KineticEvent {}

    /** The flight-phase state machine advanced (Section 2). */
    record PhaseChange(String bodyId, double bodyAge, String from, String to,
                       String reason) implements KineticEvent {}

    /** Thrust-to-weight below 1: the vehicle cannot leave the pad (RD5). */
    record LiftoffFailure(String bodyId, double bodyAge, double twr,
                          double required) implements KineticEvent {}

    // ---- seeker ----------------------------------------------------------

    /** The seeker acquired or reacquired its target (RC6). */
    record LockAcquired(String bodyId, double bodyAge, String targetId,
                        boolean reacquisition) implements KineticEvent {}

    /** The seeker lost lock and entered memory-track (RC6). */
    record LockLost(String bodyId, double bodyAge, String targetId,
                    String cause, double memoryTrackSeconds) implements KineticEvent {}

    /** Memory-track expired without reacquisition; the body is now inertial (RC6). */
    record LockExpired(String bodyId, double bodyAge, String targetId) implements KineticEvent {}

    /** A countermeasure successfully seduced the seeker onto a decoy (RF5). */
    record DecoySeduced(String bodyId, double bodyAge, String fromTarget,
                        String toDecoy, String countermeasure) implements KineticEvent {}

    // ---- orbital ---------------------------------------------------------

    /** The vehicle reached the Karman line with enough delta-v and entered the registry. */
    record OrbitInsertion(String bodyId, double bodyAge, double semiMajorAxis,
                          double periodSeconds, double achievedDeltaV) implements KineticEvent {}

    /** The vehicle reached the Karman line short of orbital velocity (RD3). */
    record InsertionFailed(String bodyId, double bodyAge, double achievedDeltaV,
                           double requiredDeltaV) implements KineticEvent {}

    /** An orbit decayed below the sustainable floor and will deorbit (RE4). */
    record OrbitDecaying(String bodyId, double bodyAge, double altitude,
                         double decayPerOrbit) implements KineticEvent {}

    /** A registry object left orbit and became an in-world descending body. */
    record Deorbit(String bodyId, double bodyAge, Vec3 entryPosition,
                   Vec3 entryVelocity, boolean commanded) implements KineticEvent {}

    // ---- failures --------------------------------------------------------

    /**
     * An invariant was breached. This is a P0 physics bug, not a gameplay event: the fix is
     * always to correct the model, never to special-case the symptom.
     */
    record InvariantBreach(String bodyId, double bodyAge, String invariant,
                           String detail) implements KineticEvent {}
}
