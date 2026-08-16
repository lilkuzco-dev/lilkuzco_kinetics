package dev.lilkuzco.kinetics.aero;

import dev.lilkuzco.kinetics.constants.Constants;

/**
 * Lift coefficient against angle of attack, including stall (RB4).
 *
 * <p>The whole point of this class is the bound. A lift model that is just
 * {@code C_L = slope * alpha} lets a guided body generate arbitrary lift by simply pitching
 * further, which is where "impossible manoeuvre" behaviour comes from - the airframe pulls a
 * turn no real one could, and nothing in the sim objects. Here C_L rises linearly to a peak
 * near 15 degrees and then collapses, so past the stall the harder you pull the less you get.
 * Invariant I5 asserts exactly that shape.
 *
 * <p>Three regimes:
 * <ul>
 *   <li><b>attached</b> - linear, slope from the profile (0.1/deg default, 91% of the
 *       thin-airfoil 2*pi/rad ideal)</li>
 *   <li><b>stalling</b> - a sharp linear collapse between the stall angle and the fully
 *       separated angle</li>
 *   <li><b>separated</b> - a bounded plateau decaying to zero at 90 degrees, where the wing
 *       is a flat plate broadside to the flow and makes no lift at all</li>
 * </ul>
 */
public final class LiftCurve {

    private final double slopePerDeg;
    private final double stallAoaDeg;
    private final double postStallAoaDeg;
    private final double postStallFraction;
    private final double clMax;

    public LiftCurve(double slopePerDeg, double stallAoaDeg,
                     double postStallAoaDeg, double postStallFraction) {
        this.slopePerDeg = slopePerDeg;
        this.stallAoaDeg = stallAoaDeg;
        this.postStallAoaDeg = postStallAoaDeg;
        this.postStallFraction = postStallFraction;
        this.clMax = slopePerDeg * stallAoaDeg;
    }

    public static LiftCurve standard(Constants k) {
        return new LiftCurve(
                k.d("aerodynamics.default_lift_slope_per_deg"),
                k.d("aerodynamics.default_stall_aoa_deg"),
                k.d("aerodynamics.post_stall_aoa_deg"),
                k.d("aerodynamics.post_stall_cl_fraction"));
    }

    /** A body with no lifting surfaces: shells, capsules, unfinned rockets. */
    public static LiftCurve none() { return new LiftCurve(0.0, 1.0, 2.0, 0.0); }

    /** Coefficient of lift at an angle of attack in degrees. Signed, bounded, continuous. */
    public double coefficientAt(double aoaDeg) {
        double a = Math.abs(aoaDeg);
        double sign = aoaDeg < 0 ? -1.0 : 1.0;
        if (a >= 90.0) return 0.0;

        if (a <= stallAoaDeg) {
            return sign * slopePerDeg * a;
        }
        if (a <= postStallAoaDeg) {
            // The collapse. Linear between C_L,max and the separated plateau.
            double t = (a - stallAoaDeg) / (postStallAoaDeg - stallAoaDeg);
            double cl = clMax + (postStallFraction * clMax - clMax) * t;
            return sign * cl;
        }
        // Separated. Decays smoothly to zero at 90 degrees - broadside, no lift.
        double t = (a - postStallAoaDeg) / (90.0 - postStallAoaDeg);
        return sign * postStallFraction * clMax * Math.cos(t * Math.PI * 0.5);
    }

    /** Peak lift coefficient - the value at the stall angle. */
    public double clMax() { return clMax; }

    public double stallAoaDeg() { return stallAoaDeg; }

    /** Whether this angle of attack is past the stall. Drives the profile-drag penalty. */
    public boolean isStalled(double aoaDeg) { return Math.abs(aoaDeg) > stallAoaDeg; }

    /**
     * How separated the flow is, 0 at the stall angle and 1 once fully separated. Used to
     * ramp the post-stall drag multiplier in rather than stepping it.
     */
    public double separation(double aoaDeg) {
        double a = Math.abs(aoaDeg);
        if (a <= stallAoaDeg) return 0.0;
        if (a >= postStallAoaDeg) return 1.0;
        return (a - stallAoaDeg) / (postStallAoaDeg - stallAoaDeg);
    }
}
