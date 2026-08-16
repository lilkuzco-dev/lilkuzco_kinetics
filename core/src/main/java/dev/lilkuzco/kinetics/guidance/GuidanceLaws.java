package dev.lilkuzco.kinetics.guidance;

import dev.lilkuzco.kinetics.env.WorldProbe;
import dev.lilkuzco.kinetics.math.Vec3;

/**
 * The cheaper guidance laws (RC5), for bodies that do not warrant a PN seeker.
 *
 * <p>All of them return a <em>commanded lateral acceleration</em> in m/s^2, in the same
 * currency PN uses, so the integrator applies the same lift, stall and g-limit ceilings to
 * every law. A cruise missile flying waypoints is subject to exactly the same turn authority
 * as an interceptor pulling a terminal manoeuvre - the difference between them is the command,
 * never the physics.
 */
public final class GuidanceLaws {

    private GuidanceLaws() {}

    /**
     * Pure pursuit: point the velocity vector straight at the target and keep it there.
     *
     * <p>Cheap, robust, and noticeably worse than PN - it produces the tail-chase curve, where
     * the pursuer keeps turning to follow rather than cutting the corner, and ends up
     * approaching from behind at low closing speed. That inferiority is the point: it is what a
     * budget seeker should feel like next to a proper one.
     *
     * @param gain how hard to correct heading error, 1/s
     */
    public static Vec3 purePursuit(Vec3 position, Vec3 velocity, Vec3 targetPosition, double gain) {
        Vec3 desired = targetPosition.sub(position);
        double speed = velocity.length();
        if (desired.lengthSq() < 1e-12 || speed < 1e-9) return Vec3.ZERO;
        Vec3 desiredDir = desired.normalized();
        Vec3 currentDir = velocity.scale(1.0 / speed);
        // Turn the velocity toward the target: the perpendicular error, scaled by speed so the
        // command is an acceleration rather than a heading rate.
        Vec3 error = desiredDir.sub(currentDir).perpendicularTo(currentDir);
        return error.scale(gain * speed);
    }

    /**
     * Fly to a waypoint under a bank-angle limit.
     *
     * <p>A body in a coordinated turn at bank angle {@code phi} pulls {@code g*tan(phi)} of
     * lateral acceleration. Capping the bank therefore caps the turn, and a 60-degree limit
     * gives 1.73 g - a realistic cruise constraint that keeps a cruise missile from cornering
     * like an interceptor even though both have the same wings.
     */
    public static Vec3 waypoint(Vec3 position, Vec3 velocity, Vec3 waypointPosition,
                                double maxBankAngleDeg, double gravity, double gain) {
        Vec3 command = purePursuit(position, velocity, waypointPosition, gain);
        double maxLateral = gravity * Math.tan(Math.toRadians(
                Math.min(maxBankAngleDeg, 89.0)));
        return command.clampLength(maxLateral);
    }

    /**
     * Hold an altitude. Proportional-derivative on altitude error, damped by vertical speed so
     * it settles instead of porpoising.
     *
     * @param kp gain on altitude error, 1/s^2
     * @param kd gain on vertical rate, 1/s
     */
    public static Vec3 altitudeHold(Vec3 position, Vec3 velocity, double targetY,
                                    double kp, double kd, double maxAccel) {
        double error = targetY - position.y();
        double rate = velocity.y();
        double command = kp * error - kd * rate;
        if (command > maxAccel) command = maxAccel;
        if (command < -maxAccel) command = -maxAccel;
        return new Vec3(0.0, command, 0.0);
    }

    /**
     * Terrain following: hold a fixed clearance over whatever is underneath, looking ahead far
     * enough to climb before arriving at it (RC5).
     *
     * <p>The look-ahead is what makes this usable. Sampling the ground directly below produces
     * a body that notices a cliff at the moment it hits it; sampling a few seconds along the
     * velocity vector lets it start climbing while there is still room.
     *
     * @param clearance   metres to hold above the terrain
     * @param lookAheadSeconds how far along the current velocity to sample
     */
    public static Vec3 terrainFollow(Vec3 position, Vec3 velocity, WorldProbe world,
                                     double clearance, double lookAheadSeconds,
                                     int searchDepth, double kp, double kd, double maxAccel) {
        Vec3 ahead = position.add(velocity.scale(lookAheadSeconds));
        int groundHere = world.groundHeight(position.x(), position.z(),
                (int) Math.ceil(position.y()), searchDepth);
        int groundAhead = world.groundHeight(ahead.x(), ahead.z(),
                (int) Math.ceil(Math.max(position.y(), ahead.y())), searchDepth);

        // Take the higher of the two: climb for the ridge you are about to reach, not the
        // valley you are currently over.
        double ground = Math.max(
                groundHere == Integer.MIN_VALUE ? Double.NEGATIVE_INFINITY : groundHere,
                groundAhead == Integer.MIN_VALUE ? Double.NEGATIVE_INFINITY : groundAhead);
        if (!Double.isFinite(ground)) return Vec3.ZERO;

        return altitudeHold(position, velocity, ground + 1.0 + clearance, kp, kd, maxAccel);
    }

    /**
     * Arrival: like a waypoint, but slowing as it closes so it settles on the point rather than
     * overshooting and orbiting it.
     */
    public static Vec3 arrival(Vec3 position, Vec3 velocity, Vec3 destination,
                               double slowingRadius, double maxAccel, double gain) {
        Vec3 offset = destination.sub(position);
        double distance = offset.length();
        if (distance < 1e-6) return velocity.scale(-gain);
        double desiredSpeed = distance < slowingRadius
                ? velocity.length() * (distance / slowingRadius)
                : velocity.length();
        Vec3 desiredVelocity = offset.scale(desiredSpeed / distance);
        return desiredVelocity.sub(velocity).scale(gain).clampLength(maxAccel);
    }
}
