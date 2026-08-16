package dev.lilkuzco.kinetics.aero;

import dev.lilkuzco.kinetics.constants.Constants;

/**
 * Transonic drag rise (RB5): a bounded multiplier on C_d as a function of Mach.
 *
 * <p>What this buys is the sound barrier. Without it a body with enough thrust just keeps
 * accelerating, and the difference between a subsonic cruise missile and a supersonic one is
 * only its engine. With it, pushing through Mach 1 costs a real drag penalty that a marginal
 * design cannot pay, and a body that does get through finds drag easing off again on the far
 * side.
 *
 * <p><b>This is not a Prandtl-Glauert implementation.</b> Prandtl-Glauert's
 * {@code 1/sqrt(1-M^2)} correction is the inspiration for the shape, but it is singular at
 * M=1 and would hand the integrator an infinity - a straight I1 violation. What is
 * implemented is a bounded empirical curve with the same qualitative behaviour: flat
 * subsonic, a smooth rise from the drag-divergence Mach to a peak just past M=1, then a
 * smooth ease onto a supersonic asymptote that stays above the subsonic value.
 *
 * <p>The transitions are cosine-smoothed rather than linear so the derivative is continuous;
 * a kink in dC_d/dM shows up as a visible twitch in a guided body's acceleration.
 */
public final class Compressibility {

    private final double divergenceMach;
    private final double peakMach;
    private final double peakMultiplier;
    private final double supersonicAsymptote;
    private final double easeMach;
    private final double incompressibleLimit;

    public Compressibility(Constants k) {
        this.divergenceMach = k.d("aerodynamics.transonic.drag_divergence_mach");
        this.peakMach = k.d("aerodynamics.transonic.peak_mach");
        this.peakMultiplier = k.d("aerodynamics.transonic.peak_multiplier");
        this.supersonicAsymptote = k.d("aerodynamics.transonic.supersonic_asymptote_multiplier");
        this.easeMach = k.d("aerodynamics.transonic.supersonic_ease_mach");
        this.incompressibleLimit = k.d("aerodynamics.transonic.incompressible_mach_limit");
    }

    /** Drag multiplier at a Mach number. Always finite, always >= 1. */
    public double dragMultiplier(double mach) {
        double m = Math.abs(mach);
        if (m <= divergenceMach) return 1.0;

        if (m <= peakMach) {
            double t = (m - divergenceMach) / (peakMach - divergenceMach);
            return 1.0 + (peakMultiplier - 1.0) * smoothstep(t);
        }
        if (m < easeMach) {
            double t = (m - peakMach) / (easeMach - peakMach);
            return peakMultiplier + (supersonicAsymptote - peakMultiplier) * smoothstep(t);
        }
        return supersonicAsymptote;
    }

    /** Below this Mach the flow is treated as incompressible and coefficients are constant. */
    public boolean isIncompressible(double mach) {
        return Math.abs(mach) < incompressibleLimit;
    }

    public double peakMach() { return peakMach; }

    public double divergenceMach() { return divergenceMach; }

    /** Cosine smoothstep on [0,1]: value and first derivative both continuous at the ends. */
    private static double smoothstep(double t) {
        if (t <= 0.0) return 0.0;
        if (t >= 1.0) return 1.0;
        return 0.5 - 0.5 * Math.cos(Math.PI * t);
    }
}
