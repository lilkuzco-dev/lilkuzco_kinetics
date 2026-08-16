package dev.lilkuzco.kinetics.ballistics;

import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.util.Rng;

/**
 * Shot dispersion from a stated circular error probable (Section 3).
 *
 * <p>CEP is the radius containing half the shots, which is the figure artillery and weapon
 * documentation actually quotes - so profiles declare CEP and this converts it into the
 * sampling parameter. For a circular normal distribution {@code CEP = 1.1774 * sigma}, hence
 * the stored 0.84932 reciprocal.
 *
 * <p>Every draw is seeded from {@code (world seed, shot index)}. The same gun firing the same
 * shot number in the same world always lands in the same place, on every machine, forever
 * (I7). Dispersion is a fixed property of the shot, not a dice roll at the moment of firing -
 * which is also what makes a golden trajectory containing dispersed fire reproducible at all.
 */
public final class Dispersion {

    private final double cepToSigma;
    private final double defaultCep;

    public Dispersion(Constants k) {
        this.cepToSigma = k.d("dispersion.cep_to_sigma");
        this.defaultCep = k.d("dispersion.default_cep");
    }

    /** Convert a stated CEP into the standard deviation of one axis. */
    public double sigmaFor(double cep) { return cep * cepToSigma; }

    public double defaultCep() { return defaultCep; }

    /**
     * Offset the aim point within the dispersion pattern.
     *
     * <p>The offset is applied perpendicular to the line of fire - a shot scatters across and
     * along the target plane, not up and down the barrel - so a long shot and a short shot with
     * the same CEP both land in a circle of the same size around where they were aimed.
     *
     * @param seed      world seed
     * @param shotIndex which shot from this weapon; distinct shots must not share a draw
     */
    public Vec3 disperseAimPoint(Vec3 origin, Vec3 aimPoint, double cep,
                                 long seed, long shotIndex) {
        if (cep <= 0.0) return aimPoint;
        Rng rng = Rng.forPurpose(seed ^ (shotIndex * 0x9E3779B97F4A7C15L), "dispersion");
        double sigma = sigmaFor(cep);

        Vec3 line = aimPoint.sub(origin);
        if (line.lengthSq() < 1e-12) return aimPoint;
        Vec3 forward = line.normalized();

        // Build a stable basis perpendicular to the line of fire. The choice of seed axis is
        // deterministic rather than arbitrary, so the pattern is reproducible (I7).
        Vec3 seedAxis = Math.abs(forward.y()) < 0.9 ? Vec3.UP : new Vec3(1, 0, 0);
        Vec3 right = forward.cross(seedAxis).normalized();
        Vec3 up = right.cross(forward).normalized();

        double a = rng.nextGaussian() * sigma;
        double b = rng.nextGaussian() * sigma;
        return aimPoint.add(right.scale(a)).add(up.scale(b));
    }

    /**
     * The fraction of shots expected within {@code radius} of the aim point, for a stated CEP.
     * Rayleigh distribution: {@code P = 1 - exp(-r^2 / (2 sigma^2))}. Used by tests to confirm
     * the sampler matches the distribution it claims.
     */
    public double fractionWithin(double radius, double cep) {
        double sigma = sigmaFor(cep);
        if (sigma <= 0.0) return 1.0;
        return 1.0 - Math.exp(-(radius * radius) / (2.0 * sigma * sigma));
    }
}
