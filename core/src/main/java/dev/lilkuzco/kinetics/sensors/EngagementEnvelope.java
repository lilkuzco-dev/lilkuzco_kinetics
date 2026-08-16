package dev.lilkuzco.kinetics.sensors;

import dev.lilkuzco.kinetics.guidance.ProportionalNavigation;
import dev.lilkuzco.kinetics.guidance.Target;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.profile.SeekerSpec;

/**
 * The launch-acceptability region (RF4): may this weapon be fired at this target, now?
 *
 * <p>A static range ring is not enough, and the reason is the difference between where a target
 * <em>is</em> and where it is <em>going</em>. A fighter at the edge of the envelope flying
 * away is not engageable even though it is technically in range; one outside the ring but
 * closing fast is. So the decision is made against the <b>predicted intercept point</b>: the
 * shot is acceptable when the point where the missile would actually meet the target lies
 * inside the envelope, and the flight time to get there fits the weapon's endurance.
 *
 * <p>Point defence is this class with a tight envelope and a short flight-time budget - the
 * geometry is identical, only the numbers differ.
 */
public final class EngagementEnvelope {

    private final SeekerSpec spec;

    public EngagementEnvelope(SeekerSpec spec) { this.spec = spec; }

    /** Why a shot was or was not acceptable. */
    public record Solution(
            boolean acceptable,
            Vec3 interceptPoint,
            double timeToIntercept,
            double interceptRange,
            String reason) {}

    /**
     * Evaluate a shot.
     *
     * @param launcherPosition where the weapon is
     * @param weaponSpeed      the average speed the weapon is expected to make good
     */
    public Solution evaluate(Vec3 launcherPosition, Vec3 weaponVelocity, double weaponSpeed,
                             Target target) {
        double currentRange = target.position().sub(launcherPosition).length();

        if (currentRange < spec.minRange()) {
            return new Solution(false, target.position(), 0.0, currentRange, String.format(
                    "inside minimum range: %.1f m against a %.1f m arming distance",
                    currentRange, spec.minRange()));
        }

        // Where the target will be when a weapon travelling at weaponSpeed could reach it.
        // Solved by iteration: guess a flight time, see where the target has moved to, repeat.
        // It converges in a handful of passes for any geometry that is actually engageable.
        double flightTime = currentRange / Math.max(weaponSpeed, 1e-6);
        Vec3 intercept = target.position();
        for (int i = 0; i < 12; i++) {
            intercept = target.predicted(flightTime).position();
            double distance = intercept.sub(launcherPosition).length();
            double next = distance / Math.max(weaponSpeed, 1e-6);
            if (Math.abs(next - flightTime) < 1e-4) { flightTime = next; break; }
            flightTime = next;
        }
        double interceptRange = intercept.sub(launcherPosition).length();

        if (interceptRange > spec.maxRange()) {
            return new Solution(false, intercept, flightTime, interceptRange, String.format(
                    "predicted intercept at %.1f m is beyond the %.1f m maximum range - the "
                    + "target is opening faster than the weapon can close",
                    interceptRange, spec.maxRange()));
        }
        if (flightTime > spec.maxFlightTime()) {
            return new Solution(false, intercept, flightTime, interceptRange, String.format(
                    "time to intercept %.2f s exceeds the %.2f s flight-time budget",
                    flightTime, spec.maxFlightTime()));
        }
        if (intercept.y() < spec.altitudeFloor()) {
            return new Solution(false, intercept, flightTime, interceptRange, String.format(
                    "predicted intercept at y=%.1f is below the %.1f m altitude floor",
                    intercept.y(), spec.altitudeFloor()));
        }
        if (intercept.y() > spec.altitudeCeiling()) {
            return new Solution(false, intercept, flightTime, interceptRange, String.format(
                    "predicted intercept at y=%.1f is above the %.1f m altitude ceiling",
                    intercept.y(), spec.altitudeCeiling()));
        }

        double crossing = crossingRateDeg(launcherPosition, target);
        if (crossing > spec.maxCrossingRateDeg()) {
            return new Solution(false, intercept, flightTime, interceptRange, String.format(
                    "target crossing at %.1f deg/s exceeds the head's %.1f deg/s limit - it "
                    + "would be notched out of the gimbal", crossing, spec.maxCrossingRateDeg()));
        }

        return new Solution(true, intercept, flightTime, interceptRange, String.format(
                "acceptable: intercept at %.1f m in %.2f s", interceptRange, flightTime));
    }

    /** Instantaneous angular rate of the line of sight from the launcher, deg/s. */
    public static double crossingRateDeg(Vec3 launcherPosition, Target target) {
        Vec3 r = target.position().sub(launcherPosition);
        double range = r.length();
        if (range < 1e-6) return 0.0;
        // The tangential component of target velocity over the range is the LOS rate.
        Vec3 tangential = target.velocity().perpendicularTo(r);
        return Math.toDegrees(tangential.length() / range);
    }

    /** Time to intercept assuming both hold their current velocity. */
    public static double timeToIntercept(Vec3 position, Vec3 velocity, Target target) {
        return ProportionalNavigation.timeToIntercept(position, velocity, target);
    }

    public SeekerSpec spec() { return spec; }
}
