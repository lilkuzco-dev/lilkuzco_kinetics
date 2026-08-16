package dev.lilkuzco.kinetics.guidance;

import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.profile.SeekerSpec;

/**
 * Proportional navigation in true 3D vector form (RC1, RC2, RC3).
 *
 * <p>{@code a_c = N * V_c * (omega x u_los)}, where {@code omega = (R x V)/(R.R)} is the
 * line-of-sight rotation rate vector, {@code V_c = -(R.V)/|R|} is closing velocity, and
 * {@code u_los} is the unit line of sight. The commanded acceleration comes out perpendicular
 * to the line of sight by construction, which is what PN means: do not chase the target, null
 * the rotation of the line to it.
 *
 * <p>The behaviour worth understanding is why that works. If the bearing to a target is not
 * changing, you are on a collision course - the same reason a ship on a constant bearing is
 * about to hit you. PN measures the bearing drift and turns hard enough to cancel it, which
 * produces the flat, apparently lazy intercept that looks nothing like chasing and gets there
 * first.
 *
 * <p><b>Where PN stops being valid.</b> It is derived for a near-collision-course launch with
 * small line-of-sight angles. Fed a target 120 degrees off the nose it commands an enormous
 * turn it cannot make, wastes its energy, and misses. RC3's answer is not to bodge the gain
 * but to refuse to run PN at all until the geometry is inside its assumptions -
 * {@link #needsBoresightAlignment} reports that, and the flight director flies a boost
 * alignment phase first.
 */
public final class ProportionalNavigation {

    private final double gain;
    private final double apnGain;
    private final double closingEpsilon;
    private final double boresightLimitDeg;
    private final boolean augmented;

    public ProportionalNavigation(Constants k, SeekerSpec seeker) {
        double n = seeker.isPresent() ? seeker.pnGain() : k.d("guidance.pn_gain_default");
        this.gain = Math.min(k.d("guidance.pn_gain_max"),
                Math.max(k.d("guidance.pn_gain_min"), n));
        this.apnGain = k.d("guidance.apn_target_accel_gain");
        this.closingEpsilon = k.d("guidance.closing_velocity_epsilon");
        this.boresightLimitDeg = k.d("guidance.boresight_align_limit_deg");
        // RC2: target-acceleration compensation is a seeker capability, not a free upgrade.
        this.augmented = seeker.quality().allowsAugmentedPn();
    }

    /** The commanded lateral acceleration, m/s^2. Zero when the geometry is degenerate. */
    public Vec3 command(Vec3 position, Vec3 velocity, Target target) {
        Vec3 r = target.position().sub(position);
        double range = r.length();
        if (range < 1e-6) return Vec3.ZERO;

        Vec3 uLos = r.scale(1.0 / range);
        Vec3 relativeVelocity = target.velocity().sub(velocity);

        // Closing velocity, positive while the range is shrinking.
        double closing = -r.dot(relativeVelocity) / range;
        if (closing < closingEpsilon) {
            // Not closing. PN divides by nothing here, but the law is meaningless against a
            // target that is opening - commanding zero is honest, and the flight director
            // decides whether to give up or re-attack.
            return Vec3.ZERO;
        }

        // LOS rotation rate vector.
        Vec3 omega = r.cross(relativeVelocity).scale(1.0 / (range * range));
        Vec3 accel = omega.cross(uLos).scale(gain * closing);

        if (augmented) {
            // RC2: add (N/2) * the component of target acceleration perpendicular to the LOS.
            // Against a manoeuvring target this removes most of the lag that plain PN pays.
            Vec3 targetPerp = target.acceleration().perpendicularTo(uLos);
            accel = accel.add(targetPerp.scale(gain * apnGain));
        }
        return accel;
    }

    /**
     * Whether the launch geometry is outside PN's derivation and needs an alignment phase
     * first (RC3).
     */
    public boolean needsBoresightAlignment(Vec3 velocity, Vec3 position, Target target) {
        Vec3 los = target.position().sub(position);
        if (los.lengthSq() < 1e-12 || velocity.lengthSq() < 1e-12) return false;
        double offBoresightDeg = Math.toDegrees(velocity.angleTo(los));
        return offBoresightDeg > boresightLimitDeg;
    }

    /**
     * Time to intercept under a constant-velocity assumption, s. Used by the launch
     * acceptability region (RF4) and by memory-track (RC6) to keep flying somewhere sensible.
     */
    public static double timeToIntercept(Vec3 position, Vec3 velocity, Target target) {
        Vec3 r = target.position().sub(position);
        Vec3 v = target.velocity().sub(velocity);
        double range = r.length();
        double closing = -r.dot(v) / Math.max(range, 1e-9);
        if (closing <= 1e-9) return Double.POSITIVE_INFINITY;
        return range / closing;
    }

    /** Predicted intercept point under constant velocity. */
    public static Vec3 predictedInterceptPoint(Vec3 position, Vec3 velocity, Target target) {
        double t = timeToIntercept(position, velocity, target);
        if (!Double.isFinite(t)) return target.position();
        return target.predicted(t).position();
    }

    public double gain() { return gain; }

    public boolean isAugmented() { return augmented; }
}
