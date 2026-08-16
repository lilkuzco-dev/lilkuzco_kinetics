package dev.lilkuzco.kinetics.math;

/**
 * Immutable double-precision 3-vector.
 *
 * <p>Every operation routes through {@link java.lang.Math} only. Invariant I7 (determinism)
 * depends on that: {@code Math.sqrt} and friends are specified to be correctly rounded or
 * within a stated ulp bound, and crucially the JIT is not permitted to substitute a
 * different result. {@code StrictMath} would be stricter still but slower; {@code Math} is
 * exact for the operations used here (sqrt is IEEE-exact) and reproducible for the rest.
 * Nothing in this class may be rewritten in terms of vectorised or native math.
 */
public record Vec3(double x, double y, double z) {

    public static final Vec3 ZERO = new Vec3(0, 0, 0);
    public static final Vec3 UP = new Vec3(0, 1, 0);

    public Vec3 add(Vec3 o) { return new Vec3(x + o.x, y + o.y, z + o.z); }

    public Vec3 sub(Vec3 o) { return new Vec3(x - o.x, y - o.y, z - o.z); }

    public Vec3 scale(double s) { return new Vec3(x * s, y * s, z * s); }

    public Vec3 neg() { return new Vec3(-x, -y, -z); }

    public double dot(Vec3 o) { return x * o.x + y * o.y + z * o.z; }

    public Vec3 cross(Vec3 o) {
        return new Vec3(
                y * o.z - z * o.y,
                z * o.x - x * o.z,
                x * o.y - y * o.x);
    }

    public double lengthSq() { return x * x + y * y + z * z; }

    public double length() { return Math.sqrt(lengthSq()); }

    /** Unit vector, or {@link #ZERO} for a zero-length input rather than NaN (I1). */
    public Vec3 normalized() {
        double len = length();
        if (len < 1e-12) return ZERO;
        return scale(1.0 / len);
    }

    /** Component of this vector parallel to {@code dir}; {@code dir} need not be unit. */
    public Vec3 projectOnto(Vec3 dir) {
        Vec3 u = dir.normalized();
        return u.scale(this.dot(u));
    }

    /** Component of this vector perpendicular to {@code dir}. */
    public Vec3 perpendicularTo(Vec3 dir) {
        return this.sub(projectOnto(dir));
    }

    /** Magnitude clamped to {@code max}, direction preserved. Used by every force clamp (I2). */
    public Vec3 clampLength(double max) {
        double lenSq = lengthSq();
        if (lenSq <= max * max || lenSq < 1e-24) return this;
        return scale(max / Math.sqrt(lenSq));
    }

    /** Angle to {@code o} in radians, in [0, pi]. Guards the acos domain against rounding. */
    public double angleTo(Vec3 o) {
        double denom = length() * o.length();
        if (denom < 1e-12) return 0.0;
        double c = dot(o) / denom;
        if (c > 1.0) c = 1.0;
        if (c < -1.0) c = -1.0;
        return Math.acos(c);
    }

    public boolean isFinite() {
        return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z);
    }

    /** Exact bit pattern, for golden-trajectory hashing (I7). Never used for display. */
    public void writeBits(java.io.DataOutput out) throws java.io.IOException {
        out.writeLong(Double.doubleToRawLongBits(x));
        out.writeLong(Double.doubleToRawLongBits(y));
        out.writeLong(Double.doubleToRawLongBits(z));
    }

    @Override
    public String toString() {
        return String.format("(%.4f, %.4f, %.4f)", x, y, z);
    }
}
