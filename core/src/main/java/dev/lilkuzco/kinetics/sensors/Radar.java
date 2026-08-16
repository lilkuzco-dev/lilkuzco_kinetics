package dev.lilkuzco.kinetics.sensors;

import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.env.WorldProbe;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.util.Rng;

/**
 * Radar detection: the range equation, the horizon, and measurement noise (RF1-RF3).
 *
 * <p><b>The fourth root is the whole physics of stealth.</b> Received power falls as
 * {@code 1/R^4} - the signal spreads on the way out and again on the way back - so detection
 * range scales as the fourth root of cross-section: {@code R_max = R_ref * (sigma/sigma_ref)^(1/4)}.
 * Reducing an airframe's RCS by a factor of 100 buys only a factor of 3.16 in range. That
 * brutal exchange rate is why stealth aircraft are shaped the way they are: nothing short of
 * enormous RCS reduction is worth anything, and the test battery asserts the exponent directly.
 *
 * <p>The horizon (RF2) beats cross-section outright. A target below the radar horizon is
 * undetectable no matter how large its signature, which is why terrain-hugging flight works
 * against any radar and why a mountain is better cover than any coating.
 *
 * <p>Noise (RF3) is deterministic given the seed. Two servers running the same engagement get
 * the same measurement error, which is what lets a golden trajectory contain a radar-guided
 * intercept at all.
 */
public final class Radar {

    private final double referenceRange;
    private final double referenceRcs;
    private final double noiseCoefficient;
    private final double minNoiseMultiplier;
    private final double horizonStep;

    public Radar(Constants k) {
        this.referenceRange = k.d("sensors.radar_reference_range");
        this.referenceRcs = k.d("sensors.radar_reference_rcs");
        this.noiseCoefficient = k.d("sensors.range_noise_coefficient");
        this.minNoiseMultiplier = k.d("sensors.pulse_integration_max_improvement");
        this.horizonStep = k.d("sensors.horizon_sample_step");
    }

    /**
     * Maximum detection range against a given radar cross-section (RF1).
     * {@code R_max = R_ref * (sigma / sigma_ref)^(1/4)}.
     */
    public double detectionRange(double rcs) {
        if (rcs <= 0.0) return 0.0;
        return referenceRange * Math.pow(rcs / referenceRcs, 0.25);
    }

    /**
     * Detection range scaled for a radar of non-reference power.
     *
     * @param powerRatio this radar's transmit power relative to the reference. Range also goes
     *                   as the fourth root of power, for the same {@code 1/R^4} reason.
     */
    public double detectionRange(double rcs, double powerRatio) {
        if (rcs <= 0.0 || powerRatio <= 0.0) return 0.0;
        return referenceRange * Math.pow((rcs / referenceRcs) * powerRatio, 0.25);
    }

    /** Whether a contact is detectable: within range, and above the horizon (RF1 + RF2). */
    public boolean detects(Vec3 radarPosition, Vec3 targetPosition, double rcs,
                           double powerRatio, WorldProbe world) {
        double range = targetPosition.sub(radarPosition).length();
        if (range > detectionRange(rcs, powerRatio)) return false;
        return hasHorizon(radarPosition, targetPosition, world);
    }

    /**
     * Radar horizon (RF2): unobstructed line of sight through the block world.
     *
     * <p>This is checked before cross-section matters, because it is absolute. No amount of
     * signature makes a target behind a ridge visible.
     */
    public boolean hasHorizon(Vec3 radarPosition, Vec3 targetPosition, WorldProbe world) {
        return world == null || world.lineOfSight(radarPosition, targetPosition, horizonStep);
    }

    /**
     * A noisy position measurement (RF3).
     *
     * <p>Error grows with range, because the same angular error subtends more distance further
     * out. Longer dwell - integrating more pulses - reduces it as {@code 1/sqrt(N)}, floored so
     * that staring at a contact forever does not give perfect knowledge.
     *
     * @param pulses number of integrated pulses; 1 is a single look
     * @param seed   deterministic seed, typically the world seed mixed with the contact id
     */
    public Vec3 measure(Vec3 radarPosition, Vec3 truePosition, int pulses, long seed) {
        double range = truePosition.sub(radarPosition).length();
        double sigma = range * noiseCoefficient * integrationFactor(pulses);
        if (sigma <= 0.0) return truePosition;

        Rng rng = Rng.forPurpose(seed, "radar-measure");
        return truePosition.add(new Vec3(
                rng.nextGaussian() * sigma,
                rng.nextGaussian() * sigma,
                rng.nextGaussian() * sigma));
    }

    /** 1-sigma position error at a range, m. */
    public double measurementSigma(double range, int pulses) {
        return range * noiseCoefficient * integrationFactor(pulses);
    }

    /** Noise multiplier from pulse integration: {@code 1/sqrt(N)}, floored (RF3). */
    public double integrationFactor(int pulses) {
        if (pulses <= 1) return 1.0;
        return Math.max(minNoiseMultiplier, 1.0 / Math.sqrt(pulses));
    }

    public double referenceRange() { return referenceRange; }

    public double referenceRcs() { return referenceRcs; }
}
