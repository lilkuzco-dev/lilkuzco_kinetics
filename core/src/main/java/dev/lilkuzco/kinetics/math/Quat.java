package dev.lilkuzco.kinetics.math;

/**
 * Unit quaternion orientation, {@code w + xi + yj + zk}.
 *
 * <p>Invariant I12 requires the norm to stay within {@code 1e-6} of unity after every
 * integration step. Quaternion integration by {@code q += 0.5 * omega * q * dt} is only
 * first-order correct and drifts off the unit sphere monotonically, so
 * {@link #renormalized()} is called every step rather than occasionally - the check and the
 * fix live together in {@link #integrate}.
 */
public record Quat(double w, double x, double y, double z) {

    public static final Quat IDENTITY = new Quat(1, 0, 0, 0);

    public static Quat fromAxisAngle(Vec3 axis, double radians) {
        Vec3 u = axis.normalized();
        if (u.lengthSq() == 0.0) return IDENTITY;
        double half = radians * 0.5;
        double s = Math.sin(half);
        return new Quat(Math.cos(half), u.x() * s, u.y() * s, u.z() * s);
    }

    /** Shortest-arc rotation taking {@code from} to {@code to}. */
    public static Quat between(Vec3 from, Vec3 to) {
        Vec3 a = from.normalized();
        Vec3 b = to.normalized();
        if (a.lengthSq() == 0.0 || b.lengthSq() == 0.0) return IDENTITY;
        double d = a.dot(b);
        if (d >= 1.0 - 1e-12) return IDENTITY;
        if (d <= -1.0 + 1e-12) {
            // Antiparallel: any perpendicular axis is a valid pi rotation. Pick one
            // deterministically so I7 holds - never an arbitrary or random axis.
            Vec3 axis = Math.abs(a.x()) < 0.9 ? new Vec3(1, 0, 0) : new Vec3(0, 1, 0);
            return fromAxisAngle(a.cross(axis), Math.PI);
        }
        Vec3 axis = a.cross(b);
        double s = Math.sqrt((1.0 + d) * 2.0);
        return new Quat(s * 0.5, axis.x() / s, axis.y() / s, axis.z() / s).renormalized();
    }

    public double norm() { return Math.sqrt(w * w + x * x + y * y + z * z); }

    public Quat renormalized() {
        double n = norm();
        if (n < 1e-12) return IDENTITY;
        double inv = 1.0 / n;
        return new Quat(w * inv, x * inv, y * inv, z * inv);
    }

    public Quat multiply(Quat o) {
        return new Quat(
                w * o.w - x * o.x - y * o.y - z * o.z,
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w);
    }

    public Quat conjugate() { return new Quat(w, -x, -y, -z); }

    /** Rotate a vector by this orientation. */
    public Vec3 rotate(Vec3 v) {
        // v' = v + 2w(q_v x v) + 2(q_v x (q_v x v)) - the standard expansion, which avoids
        // building a matrix and keeps the operation count low in the hot loop.
        Vec3 qv = new Vec3(x, y, z);
        Vec3 t = qv.cross(v).scale(2.0);
        return v.add(t.scale(w)).add(qv.cross(t));
    }

    /** Body forward axis (+Z) expressed in world coordinates. */
    public Vec3 forward() { return rotate(new Vec3(0, 0, 1)); }

    /**
     * Advance orientation by angular velocity {@code omega} (rad/s, world frame) over
     * {@code dt}, then renormalize. Renormalization is unconditional: I12 is maintained by
     * construction, not by a later check that could be forgotten.
     */
    public Quat integrate(Vec3 omega, double dt) {
        Quat wq = new Quat(0, omega.x(), omega.y(), omega.z());
        Quat dq = wq.multiply(this).scale(0.5 * dt);
        return new Quat(w + dq.w, x + dq.x, y + dq.y, z + dq.z).renormalized();
    }

    public Quat scale(double s) { return new Quat(w * s, x * s, y * s, z * s); }

    /** Spherical linear interpolation, used by commanded slews (RE6). */
    public Quat slerp(Quat target, double t) {
        double d = w * target.w + x * target.x + y * target.y + z * target.z;
        Quat end = target;
        if (d < 0.0) { end = new Quat(-target.w, -target.x, -target.y, -target.z); d = -d; }
        if (d > 0.9995) {
            return new Quat(w + (end.w - w) * t, x + (end.x - x) * t,
                    y + (end.y - y) * t, z + (end.z - z) * t).renormalized();
        }
        double theta = Math.acos(Math.min(1.0, Math.max(-1.0, d)));
        double sinTheta = Math.sin(theta);
        double a = Math.sin((1 - t) * theta) / sinTheta;
        double b = Math.sin(t * theta) / sinTheta;
        return new Quat(a * w + b * end.w, a * x + b * end.x,
                a * y + b * end.y, a * z + b * end.z).renormalized();
    }

    /** Angle in radians between this orientation and {@code o}. */
    public double angleTo(Quat o) {
        double d = Math.abs(w * o.w + x * o.x + y * o.y + z * o.z);
        return 2.0 * Math.acos(Math.min(1.0, d));
    }

    public boolean isFinite() {
        return Double.isFinite(w) && Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z);
    }

    public void writeBits(java.io.DataOutput out) throws java.io.IOException {
        out.writeLong(Double.doubleToRawLongBits(w));
        out.writeLong(Double.doubleToRawLongBits(x));
        out.writeLong(Double.doubleToRawLongBits(y));
        out.writeLong(Double.doubleToRawLongBits(z));
    }
}
