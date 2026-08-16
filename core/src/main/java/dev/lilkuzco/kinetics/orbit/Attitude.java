package dev.lilkuzco.kinetics.orbit;

import dev.lilkuzco.kinetics.math.Quat;
import dev.lilkuzco.kinetics.math.Vec3;

/**
 * Satellite attitude: quaternion orientation plus angular velocity (RE6).
 *
 * <p>Attitude gates capability. A camera pointed at the stars sees nothing useful, and an
 * antenna pointed away from the ground carries no traffic - so a satellite that cannot hold
 * attitude cannot do its job, and holding attitude costs something. The two modes are the two
 * real answers to that problem:
 *
 * <ul>
 *   <li><b>Spin stabilised</b> - the satellite is a gyroscope. Cheap, needs no active control,
 *       and rigidly holds one axis; the price is that it can never point anywhere else. Real
 *       spin-stabilised craft carry despun platforms for exactly this reason.</li>
 *   <li><b>Three-axis</b> - the satellite slews to whatever attitude is commanded, at a bounded
 *       rate. Far more capable, and the slew rate is what stops it snapping instantly from one
 *       target to the next.</li>
 * </ul>
 *
 * <p>Invariant I12 applies here as much as to an in-flight body: the quaternion is renormalised
 * every step and angular velocity is capped, so a satellite can neither NaN its orientation nor
 * spin up without bound.
 */
public final class Attitude {

    public enum Mode {
        /** Gyroscopic hold about the spin axis. Cannot be commanded elsewhere. */
        SPIN_STABILIZED,
        /** Commanded slew to an arbitrary attitude at a bounded rate. */
        THREE_AXIS
    }

    private final Mode mode;
    private final double maxSlewRateDeg;
    private Quat orientation;
    private Vec3 angularVelocity;
    private Quat commanded;

    private Attitude(Mode mode, Quat initial, Vec3 angularVelocity, double maxSlewRateDeg) {
        this.mode = mode;
        this.orientation = initial.renormalized();
        this.angularVelocity = angularVelocity;
        this.maxSlewRateDeg = maxSlewRateDeg;
        this.commanded = this.orientation;
    }

    /** Spin about {@code spinAxis} at {@code rateDegPerSecond}. */
    public static Attitude spinStabilized(Quat initial, Vec3 spinAxis, double rateDegPerSecond) {
        return new Attitude(Mode.SPIN_STABILIZED, initial,
                spinAxis.normalized().scale(Math.toRadians(rateDegPerSecond)), 0.0);
    }

    public static Attitude threeAxis(Quat initial, double maxSlewRateDeg) {
        return new Attitude(Mode.THREE_AXIS, initial, Vec3.ZERO, maxSlewRateDeg);
    }

    /** Command a new attitude. Ignored by a spin-stabilised craft, which cannot comply. */
    public boolean command(Quat target) {
        if (mode == Mode.SPIN_STABILIZED) return false;
        this.commanded = target.renormalized();
        return true;
    }

    /** Command the craft to point its forward axis along a direction (nadir, a ground station). */
    public boolean pointAt(Vec3 direction) {
        if (mode == Mode.SPIN_STABILIZED) return false;
        return command(Quat.between(new Vec3(0, 0, 1), direction));
    }

    /** Advance by {@code dt} seconds. */
    public void advance(double dt) {
        if (mode == Mode.SPIN_STABILIZED) {
            orientation = orientation.integrate(angularVelocity, dt);
            return;
        }
        double remaining = orientation.angleTo(commanded);
        if (remaining < 1e-9) {
            angularVelocity = Vec3.ZERO;
            return;
        }
        double maxStep = Math.toRadians(maxSlewRateDeg) * dt;
        double t = maxStep >= remaining ? 1.0 : maxStep / remaining;
        Quat next = orientation.slerp(commanded, t);
        // Report the rotation actually performed, so consumers see a real angular rate.
        double achieved = orientation.angleTo(next);
        angularVelocity = dt > 0.0 ? axisOf(orientation, next).scale(achieved / dt) : Vec3.ZERO;
        orientation = next;
    }

    /** Whether the forward axis is within {@code toleranceDeg} of a direction. */
    public boolean isPointingAt(Vec3 direction, double toleranceDeg) {
        if (direction.lengthSq() < 1e-18) return false;
        return Math.toDegrees(orientation.forward().angleTo(direction)) <= toleranceDeg;
    }

    private static Vec3 axisOf(Quat from, Quat to) {
        Quat delta = to.multiply(from.conjugate()).renormalized();
        Vec3 axis = new Vec3(delta.x(), delta.y(), delta.z());
        return axis.lengthSq() < 1e-18 ? Vec3.ZERO : axis.normalized();
    }

    public Mode mode() { return mode; }

    public Quat orientation() { return orientation; }

    public Vec3 angularVelocity() { return angularVelocity; }

    public Quat commanded() { return commanded; }

    /** Whether the commanded attitude has been reached. */
    public boolean settled() {
        return mode == Mode.SPIN_STABILIZED || orientation.angleTo(commanded) < 1e-6;
    }
}
