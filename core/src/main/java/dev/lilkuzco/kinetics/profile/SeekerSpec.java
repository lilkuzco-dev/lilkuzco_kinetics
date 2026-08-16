package dev.lilkuzco.kinetics.profile;

/**
 * Seeker head and engagement envelope (RC1-RC6, RF4, RF5).
 *
 * <p>The field-of-view cone is what stops seekers being omniscient. A seeker whose target
 * leaves the cone loses lock, and losing lock is the mechanic that makes terrain masking and
 * notching real tactics rather than flavour text - a pilot who puts a ridge between
 * themselves and the missile has actually done something.
 *
 * @param quality             gates augmented PN: only good seekers get target-acceleration
 *                            compensation (RC2)
 * @param fieldOfViewDeg      gimbal half-angle. Target outside this cone = lock lost (RC6)
 * @param pnGain              navigation constant N, clamped to [2,6] at load (RC1)
 * @param memoryTrackSeconds  coast time toward the last predicted intercept after losing lock
 *                            before going inertial (RC6)
 * @param minRange            arming distance; inside this the warhead cannot be used (RF4)
 * @param maxRange            kinematic reach (RF4)
 * @param altitudeFloor       lowest engageable target altitude, m above datum (RF4)
 * @param altitudeCeiling     highest engageable target altitude (RF4)
 * @param maxCrossingRateDeg  fastest angular crossing the head can track, deg/s (RF4)
 * @param maxFlightTime       time-to-intercept budget; a solution longer than this is outside
 *                            the launch-acceptability region (RF4)
 * @param flareResistance     0..1, resistance to IR decoy seduction (RF5)
 * @param chaffResistance     0..1, resistance to RF decoy seduction (RF5)
 */
public record SeekerSpec(
        Quality quality,
        double fieldOfViewDeg,
        double pnGain,
        double memoryTrackSeconds,
        double minRange,
        double maxRange,
        double altitudeFloor,
        double altitudeCeiling,
        double maxCrossingRateDeg,
        double maxFlightTime,
        double flareResistance,
        double chaffResistance) {

    public enum Quality {
        /** Plain PN only, narrow cone, poor counter-countermeasures. */
        CHEAP,
        /** Plain PN, decent cone. */
        STANDARD,
        /** Augmented PN against manoeuvring targets (RC2), wide cone, resistant. */
        ADVANCED;

        /** Whether this head may use augmented PN (RC2). */
        public boolean allowsAugmentedPn() { return this == ADVANCED; }
    }

    /** No seeker: ballistic and unguided bodies. */
    public static SeekerSpec none() {
        return new SeekerSpec(Quality.CHEAP, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public boolean isPresent() { return fieldOfViewDeg > 0.0; }

    /** Whether a target at this range and altitude is inside the static envelope (RF4). */
    public boolean withinEnvelope(double range, double targetAltitude) {
        return range >= minRange && range <= maxRange
                && targetAltitude >= altitudeFloor && targetAltitude <= altitudeCeiling;
    }
}
