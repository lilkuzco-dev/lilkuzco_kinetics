package dev.lilkuzco.kinetics.profile;

import dev.lilkuzco.kinetics.aero.Compressibility;
import dev.lilkuzco.kinetics.aero.DragPolar;
import dev.lilkuzco.kinetics.aero.LiftCurve;
import dev.lilkuzco.kinetics.aero.Aerodynamics;
import dev.lilkuzco.kinetics.constants.Constants;

/**
 * The shape of a body: what the air does to it, and what it can survive.
 *
 * @param referenceArea      drag reference area, m^2
 * @param wingArea           lift reference area, m^2 (equal to reference area for a body with
 *                           no distinct wing; small for a finned missile)
 * @param cd0                zero-lift drag coefficient
 * @param aspectRatio        wing aspect ratio; {@code Infinity} for a body with no lifting
 *                           surface, which zeroes the induced term rather than special-casing it
 * @param oswaldEfficiency   Oswald span efficiency, 0.7-0.85 typical (RB4b)
 * @param liftSlopePerDeg    linear-regime lift-curve slope (RB4)
 * @param stallAoaDeg        angle of attack at C_L,max (RB4)
 * @param postStallAoaDeg    angle by which flow is fully separated (RB4)
 * @param postStallClFraction fraction of C_L,max retained when separated (RB4)
 * @param gLimitG            acceleration clamp in g (I2)
 * @param qMaxPa             dynamic-pressure structural limit (RB6)
 * @param noseRadius         effective nose radius for reentry heating (RE7)
 * @param overheatThreshold  heating rate at which the overheat event fires (RE7)
 * @param rcs                radar cross-section, m^2 (RF1)
 */
public record Airframe(
        double referenceArea,
        double wingArea,
        double cd0,
        double aspectRatio,
        double oswaldEfficiency,
        double liftSlopePerDeg,
        double stallAoaDeg,
        double postStallAoaDeg,
        double postStallClFraction,
        double gLimitG,
        double qMaxPa,
        double noseRadius,
        double overheatThreshold,
        double rcs) {

    /** Build the aerodynamic model this airframe describes. */
    public Aerodynamics aerodynamics(Constants k) {
        LiftCurve lift = wingArea <= 0.0 || liftSlopePerDeg <= 0.0
                ? LiftCurve.none()
                : new LiftCurve(liftSlopePerDeg, stallAoaDeg, postStallAoaDeg, postStallClFraction);
        DragPolar polar = new DragPolar(cd0, aspectRatio, oswaldEfficiency,
                k.d("aerodynamics.post_stall_cd_multiplier"));
        return new Aerodynamics(lift, polar, new Compressibility(k));
    }

    /** Acceleration clamp in m/s^2 (I2). */
    public double gLimitAccel(Constants k) {
        return gLimitG * k.d("gravity.g0");
    }

    /** A simple bluff body: shells, capsules, debris. No lifting surface. */
    public static Airframe bluff(double mass, double area, double cd, Constants k) {
        return new Airframe(area, 0.0, cd, Double.POSITIVE_INFINITY,
                k.d("aerodynamics.oswald_efficiency_default"),
                0.0, k.d("aerodynamics.default_stall_aoa_deg"),
                k.d("aerodynamics.post_stall_aoa_deg"),
                k.d("aerodynamics.post_stall_cl_fraction"),
                k.d("limits.g_limit_default"),
                k.d("limits.q_max_default"),
                k.d("reentry.default_nose_radius"),
                k.d("reentry.overheat_threshold_default"),
                k.d("sensors.rcs_bins.cruise_missile"));
    }
}
