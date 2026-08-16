package dev.lilkuzco.kinetics.aero;

import dev.lilkuzco.kinetics.math.Vec3;

/**
 * Drag and lift force vectors from an airspeed vector (RB2, RB4, RB4b, RB5).
 *
 * <p>Two geometric guarantees hold by construction here, and both are load-bearing:
 * <ul>
 *   <li><b>Drag is exactly antiparallel to airspeed.</b> Not to velocity - to airspeed, which
 *       is velocity minus wind. That is the I5 wording and it matters the moment RB7 wind is
 *       switched on, because drag against ground velocity in a crosswind would push a body
 *       sideways for no reason.</li>
 *   <li><b>Lift is exactly perpendicular to airspeed.</b> It is built as the component of the
 *       body's forward axis perpendicular to the airflow, so its dot product with airspeed is
 *       zero analytically, not approximately. Lift therefore does no work, and I3's energy
 *       ledger stays closed no matter how violently a body manoeuvres.</li>
 * </ul>
 */
public final class Aerodynamics {

    private final LiftCurve liftCurve;
    private final DragPolar dragPolar;
    private final Compressibility compressibility;

    public Aerodynamics(LiftCurve liftCurve, DragPolar dragPolar, Compressibility compressibility) {
        this.liftCurve = liftCurve;
        this.dragPolar = dragPolar;
        this.compressibility = compressibility;
    }

    /**
     * Aerodynamic state and forces for one instant.
     *
     * @param airspeed        velocity relative to the air mass (m/s)
     * @param forward         body forward axis, unit, world frame
     * @param density         air density (kg/m^3)
     * @param machNumber      airspeed / local speed of sound
     * @param referenceArea   drag reference area (m^2)
     * @param wingArea        lift reference area (m^2)
     */
    public AeroResult compute(Vec3 airspeed, Vec3 forward, double density,
                              double machNumber, double referenceArea, double wingArea) {
        double speed = airspeed.length();
        if (density <= 0.0 || speed < 1e-9) {
            // Vacuum or at rest with the air: no aerodynamic force of any kind. This is the
            // Moon case and it needs no special handling downstream.
            return new AeroResult(Vec3.ZERO, Vec3.ZERO, 0.0, 0.0, 0.0, 0.0, machNumber, false);
        }

        Vec3 airDir = airspeed.scale(1.0 / speed);
        double q = 0.5 * density * speed * speed;

        // Angle of attack: between where the nose points and where the body is actually going
        // through the air. Signed by whether the nose is above or below the flight path.
        double aoaRad = forward.angleTo(airDir);
        double aoaDeg = Math.toDegrees(aoaRad);
        Vec3 liftAxis = forward.perpendicularTo(airDir);
        if (liftAxis.lengthSq() < 1e-18) {
            // Nose exactly on the airflow: no angle of attack, so no lift direction is defined.
            aoaDeg = 0.0;
            liftAxis = Vec3.ZERO;
        } else {
            liftAxis = liftAxis.normalized();
        }

        double cl = liftCurve.coefficientAt(aoaDeg);
        double separation = liftCurve.separation(aoaDeg);
        double cd = dragPolar.coefficientAt(cl, separation) * compressibility.dragMultiplier(machNumber);

        // F = 1/2 rho v^2 C A, with q already carrying the 1/2 rho v^2.
        Vec3 drag = airDir.scale(-q * cd * referenceArea);
        Vec3 lift = liftAxis.scale(q * cl * wingArea);

        return new AeroResult(drag, lift, aoaDeg, cl, cd, q, machNumber,
                liftCurve.isStalled(aoaDeg));
    }

    /**
     * Closed-form terminal velocity (RB3): {@code v_t = sqrt(2 m g / (rho C_d A))}.
     *
     * <p>This is the reference the integrator is checked against, so it is written out from
     * the equation rather than measured from a simulation - a self-check that measured its
     * own output would prove nothing.
     */
    public static double terminalVelocity(double mass, double gravity, double density,
                                          double cd, double area) {
        double denom = density * cd * area;
        if (denom <= 0.0) return Double.POSITIVE_INFINITY; // vacuum: nothing to balance gravity
        return Math.sqrt(2.0 * mass * gravity / denom);
    }

    public LiftCurve liftCurve() { return liftCurve; }

    public DragPolar dragPolar() { return dragPolar; }

    public Compressibility compressibility() { return compressibility; }

    /** Forces and the aerodynamic state that produced them. */
    public record AeroResult(
            Vec3 dragForce,
            Vec3 liftForce,
            double angleOfAttackDeg,
            double cl,
            double cd,
            double dynamicPressure,
            double mach,
            boolean stalled) {

        public Vec3 totalForce() { return dragForce.add(liftForce); }
    }
}
