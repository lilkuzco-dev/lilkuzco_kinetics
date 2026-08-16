package dev.lilkuzco.kinetics.guidance;

import dev.lilkuzco.kinetics.math.Vec3;

/**
 * What a seeker is tracking.
 *
 * <p>Acceleration is carried because augmented PN needs it (RC2) - and because carrying it
 * explicitly makes it obvious that a cheap seeker which is not allowed to use it is being
 * denied information it could physically have, rather than information the sim lacks.
 *
 * @param signature radar cross-section or IR class, m^2 (RF1, RF5)
 * @param isDecoy   whether this contact is a countermeasure presenting a legitimate signature
 */
public record Target(
        String id,
        Vec3 position,
        Vec3 velocity,
        Vec3 acceleration,
        double signature,
        boolean isDecoy) {

    public static Target stationary(String id, Vec3 position) {
        return new Target(id, position, Vec3.ZERO, Vec3.ZERO, 1.0, false);
    }

    public static Target moving(String id, Vec3 position, Vec3 velocity) {
        return new Target(id, position, velocity, Vec3.ZERO, 1.0, false);
    }

    public Target withSignature(double rcs) {
        return new Target(id, position, velocity, acceleration, rcs, isDecoy);
    }

    /** Dead-reckon forward. Used by memory-track (RC6) and intercept prediction (RF4). */
    public Target predicted(double seconds) {
        Vec3 p = position.add(velocity.scale(seconds))
                .add(acceleration.scale(0.5 * seconds * seconds));
        Vec3 v = velocity.add(acceleration.scale(seconds));
        return new Target(id, p, v, acceleration, signature, isDecoy);
    }
}
