package dev.lilkuzco.kinetics.aero;

import dev.lilkuzco.kinetics.constants.Constants;

/**
 * The drag polar (RB4b): {@code C_D = C_D0 + C_L^2 / (pi * AR * e)}.
 *
 * <p>Induced drag is the term that makes lift cost something. Without it, a winged body can
 * hold a maximum-C_L turn forever at no energetic price, which is free manoeuvring - the same
 * class of cheat as free energy, and against the spirit of I3 even though no thrust is
 * involved. With it, pulling hard bleeds speed, speed loss reduces available lift, and a
 * sustained hard turn decays on its own. That feedback loop is what makes air combat
 * geometry mean anything, and the test battery asserts the speed bleed directly.
 *
 * <p>The quadratic dependence matters: doubling C_L quadruples the induced penalty. High
 * aspect ratio (a long thin wing) and high Oswald efficiency both reduce it, which is why
 * gliders look the way they do and missiles do not.
 */
public final class DragPolar {

    private final double cd0;
    private final double aspectRatio;
    private final double oswaldEfficiency;
    private final double postStallMultiplier;
    private final double inducedDenominator;

    public DragPolar(double cd0, double aspectRatio, double oswaldEfficiency,
                     double postStallMultiplier) {
        this.cd0 = cd0;
        this.aspectRatio = aspectRatio;
        this.oswaldEfficiency = oswaldEfficiency;
        this.postStallMultiplier = postStallMultiplier;
        this.inducedDenominator = Math.PI * aspectRatio * oswaldEfficiency;
    }

    /** A body with no lifting surfaces: parasite drag only, no induced term. */
    public static DragPolar bluff(double cd0, Constants k) {
        return new DragPolar(cd0, Double.POSITIVE_INFINITY,
                k.d("aerodynamics.oswald_efficiency_default"),
                k.d("aerodynamics.post_stall_cd_multiplier"));
    }

    public static DragPolar winged(double cd0, double aspectRatio, Constants k) {
        return new DragPolar(cd0, aspectRatio,
                k.d("aerodynamics.oswald_efficiency_default"),
                k.d("aerodynamics.post_stall_cd_multiplier"));
    }

    /**
     * Total drag coefficient.
     *
     * @param cl current lift coefficient
     * @param separation flow separation 0..1 from {@link LiftCurve#separation}; ramps the
     *                   profile-drag penalty in rather than stepping it at the stall angle
     */
    public double coefficientAt(double cl, double separation) {
        double parasite = cd0 * (1.0 + (postStallMultiplier - 1.0) * separation);
        double induced = inducedDenominator <= 0.0 || Double.isInfinite(inducedDenominator)
                ? 0.0
                : (cl * cl) / inducedDenominator;
        return parasite + induced;
    }

    /** Zero-lift drag coefficient. */
    public double cd0() { return cd0; }

    public double aspectRatio() { return aspectRatio; }

    public double oswaldEfficiency() { return oswaldEfficiency; }

    /** Induced drag alone, for the audit and for tests that need to see the term isolated. */
    public double inducedComponent(double cl) {
        if (inducedDenominator <= 0.0 || Double.isInfinite(inducedDenominator)) return 0.0;
        return (cl * cl) / inducedDenominator;
    }
}
