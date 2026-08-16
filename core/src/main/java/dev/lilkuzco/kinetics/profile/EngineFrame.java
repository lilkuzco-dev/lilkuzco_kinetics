package dev.lilkuzco.kinetics.profile;

import dev.lilkuzco.kinetics.constants.Constants;

/**
 * The conversion from a declared specific impulse to the exhaust velocity the sim uses.
 *
 * <p>This exists as a type rather than a loose pair of doubles so that the exhaust-velocity
 * scale cannot be forgotten at a call site. Every mass-flow, thrust and delta-v computation in
 * the library goes through {@link #exhaustVelocity}, so there is exactly one place where a real
 * Isp becomes a game exhaust velocity.
 *
 * <p><b>Why a scale exists at all.</b> Profiles declare real specific impulse - 311 s for
 * kerolox, 450 s for hydrolox - because those are the numbers an engine is known by. But the
 * empire's planet is 18.6 times smaller than Earth, so its orbital velocity is 4.2 times lower,
 * and against unscaled real exhaust velocities the delta-v budget would need a mass ratio of
 * only 2.1. Orbit would be trivial and staging pointless. Dividing exhaust velocity by 4.215
 * restores the real mass ratio of 21.8 - so the engines keep their recognisable numbers and
 * reaching orbit keeps its real difficulty (RD3).
 */
public record EngineFrame(double g0, double exhaustVelocityScale) {

    public static EngineFrame of(Constants k) {
        return new EngineFrame(k.d("gravity.g0"), k.d("propulsion.exhaust_velocity_scale"));
    }

    /** Effective exhaust velocity for a declared specific impulse, m/s. */
    public double exhaustVelocity(double ispSeconds) {
        if (exhaustVelocityScale <= 0.0) return ispSeconds * g0;
        return ispSeconds * g0 / exhaustVelocityScale;
    }

    /** The unscaled, real-world exhaust velocity, for the scale audit and for documentation. */
    public double realExhaustVelocity(double ispSeconds) { return ispSeconds * g0; }
}
