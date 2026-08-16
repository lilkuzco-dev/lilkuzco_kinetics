package dev.lilkuzco.kinetics.env;

import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.math.Vec3;

/**
 * Deterministic, seeded wind (RB7). Off by default.
 *
 * <p>Wind here is a <em>field</em>, not noise. It is a pure function of
 * {@code (world seed, position, world time)} with no internal state and no per-tick random
 * draws, which is the only way it can coexist with I7: replaying the same launch on another
 * machine must produce the same trajectory, and a turbulence generator that ticks would make
 * that impossible.
 *
 * <p>The structure is deliberately low-frequency - a 512 m horizontal wavelength and a 40 m
 * vertical band - so wind reads as a regional weather condition a player can learn and
 * compensate for, rather than a jitter that just adds variance to every shot.
 */
public final class WindField {

    private final boolean enabled;
    private final double baseSpeed;
    private final double shearExponent;
    private final double bandHeight;
    private final double spatialWavelength;
    private final double timePeriod;
    private final double seedPhaseX;
    private final double seedPhaseZ;

    private WindField(Constants k, boolean enabled, long worldSeed) {
        this.enabled = enabled;
        this.baseSpeed = k.d("wind.base_speed");
        this.shearExponent = k.d("wind.shear_exponent");
        this.bandHeight = k.d("wind.band_height");
        this.spatialWavelength = k.d("wind.spatial_wavelength");
        this.timePeriod = k.d("wind.time_period");
        // Two fixed phase offsets derived from the world seed. Mixing with the golden-ratio
        // constant spreads adjacent seeds apart, so seed 1 and seed 2 are not near-identical
        // weather. Computed once at construction - the field itself stays stateless.
        long h = worldSeed * 0x9E3779B97F4A7C15L;
        this.seedPhaseX = ((h >>> 11) * 0x1.0p-53) * 2.0 * Math.PI;
        this.seedPhaseZ = (((h * 0xBF58476D1CE4E5B9L) >>> 11) * 0x1.0p-53) * 2.0 * Math.PI;
    }

    public static WindField disabled(Constants k) { return new WindField(k, false, 0L); }

    public static WindField seeded(Constants k, long worldSeed) {
        return new WindField(k, true, worldSeed);
    }

    public boolean isEnabled() { return enabled; }

    /**
     * Wind velocity in m/s at a world position and time. Horizontal only - vertical gusts are
     * deliberately absent, because a vertical component large enough to notice would fight the
     * gravity-turn pitch program for no gameplay benefit.
     *
     * @param position world position, y measured from the sea-level datum
     * @param worldTimeSeconds monotonic world time
     */
    public Vec3 at(Vec3 position, double worldTimeSeconds) {
        if (!enabled) return Vec3.ZERO;

        double altitude = Math.max(0.0, position.y());

        // Power-law shear: wind strengthens with height. Referenced to the band height so the
        // constant reads as "the speed at 40 m", which is a number a designer can reason about.
        double shear = Math.pow((altitude + 1.0) / bandHeight, shearExponent);

        double kx = 2.0 * Math.PI / spatialWavelength;
        double kt = 2.0 * Math.PI / timePeriod;
        double kb = 2.0 * Math.PI / bandHeight;

        // Two out-of-phase components so the direction rotates with altitude (an Ekman-like
        // spiral) instead of the whole column blowing one way.
        double phase = kx * position.x() + seedPhaseX + kt * worldTimeSeconds;
        double bandPhase = kb * altitude;

        double vx = Math.sin(phase) * Math.cos(bandPhase);
        double vz = Math.cos(kx * position.z() + seedPhaseZ + kt * worldTimeSeconds)
                * Math.sin(bandPhase + Math.PI / 4.0);

        return new Vec3(vx, 0.0, vz).scale(baseSpeed * shear);
    }
}
