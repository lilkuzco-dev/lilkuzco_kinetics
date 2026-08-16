package dev.lilkuzco.kinetics.util;

/**
 * SplitMix64 - the deterministic random source for the whole library.
 *
 * <p>Every stochastic thing kinetics does (dispersion, sensor noise, countermeasure rolls,
 * wind field, fuzz scenarios) draws from here. {@code java.util.Random} would also be
 * reproducible, but SplitMix64 is written out in full below, so the sequence is fixed by
 * this file rather than by a JDK implementation detail - which is what invariant I7 actually
 * needs. The algorithm is Steele, Lea and Flood's, published in the SplittableRandom paper.
 *
 * <p>Instances are cheap and are meant to be created per (seed, purpose) rather than shared,
 * so that adding a new consumer never shifts an existing consumer's draw sequence. Use
 * {@link #forPurpose} to derive an independent stream.
 */
public final class Rng {

    private long state;

    public Rng(long seed) { this.state = seed; }

    /**
     * Derive an independent stream from a base seed and a purpose label. Two subsystems using
     * the same world seed must not draw from the same sequence, or adding a feature would
     * silently change existing golden trajectories.
     */
    public static Rng forPurpose(long seed, String purpose) {
        long h = 0xcbf29ce484222325L;
        for (int i = 0; i < purpose.length(); i++) {
            h ^= purpose.charAt(i);
            h *= 0x100000001b3L;
        }
        return new Rng(seed ^ h);
    }

    public long nextLong() {
        state += 0x9E3779B97F4A7C15L;
        long z = state;
        z = (z ^ (z >>> 30)) * 0xBF58476D1CE4E5B9L;
        z = (z ^ (z >>> 27)) * 0x94D049BB133111EBL;
        return z ^ (z >>> 31);
    }

    /** Uniform in [0, 1). Uses the top 53 bits, the full mantissa of a double. */
    public double nextDouble() {
        return (nextLong() >>> 11) * 0x1.0p-53;
    }

    /** Uniform in [min, max). */
    public double range(double min, double max) {
        return min + (max - min) * nextDouble();
    }

    /** Uniform in [0, bound). */
    public int nextInt(int bound) {
        if (bound <= 0) throw new IllegalArgumentException("bound must be positive");
        return (int) Math.floorMod(nextLong(), (long) bound);
    }

    public boolean nextBoolean() { return (nextLong() & 1L) != 0L; }

    /** Standard normal via Box-Muller. Both draws are consumed so the stream stays aligned. */
    public double nextGaussian() {
        double u1 = nextDouble();
        double u2 = nextDouble();
        if (u1 < 1e-300) u1 = 1e-300;
        return Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
    }

    /** A roll succeeding with probability {@code p}, clamped to [0,1]. */
    public boolean chance(double p) {
        if (p <= 0.0) return false;
        if (p >= 1.0) return true;
        return nextDouble() < p;
    }
}
