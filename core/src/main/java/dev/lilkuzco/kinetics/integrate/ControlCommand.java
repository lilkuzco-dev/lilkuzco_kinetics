package dev.lilkuzco.kinetics.integrate;

import dev.lilkuzco.kinetics.math.Vec3;

/**
 * What guidance is asking the airframe to do this tick.
 *
 * <p>Note what this record does <em>not</em> contain: a force. Guidance may not inject
 * acceleration into the integrator directly. It asks for a lateral acceleration or a heading,
 * and {@link Integrator} works out whether the airframe can actually deliver that - which
 * depends on dynamic pressure, the stall limit and the g-limit. That indirection is what makes
 * invariant I6 structural: a body cannot out-turn its own aerodynamics because the turn is
 * produced by its aerodynamics, not handed to it.
 *
 * <p>The two modes correspond to the two ways real guidance talks to an airframe: "hold this
 * attitude" (a gravity-turn pitch program, altitude hold, a waypoint leg) and "pull this many
 * g perpendicular to the line of sight" (proportional navigation).
 */
public record ControlCommand(Mode mode, Vec3 target, double throttle) {

    public enum Mode {
        /** No steering input. The body weathervanes into the airflow. */
        COAST,
        /** {@code target} is a direction to point the nose. */
        DIRECTION,
        /** {@code target} is a commanded acceleration vector, m/s^2 (PN and friends). */
        LATERAL_ACCEL
    }

    public static ControlCommand coast() {
        return new ControlCommand(Mode.COAST, Vec3.ZERO, 0.0);
    }

    public static ControlCommand coastWithThrust(double throttle) {
        return new ControlCommand(Mode.COAST, Vec3.ZERO, throttle);
    }

    public static ControlCommand pointAt(Vec3 direction, double throttle) {
        return new ControlCommand(Mode.DIRECTION, direction.normalized(), throttle);
    }

    public static ControlCommand accelerate(Vec3 lateralAccel, double throttle) {
        return new ControlCommand(Mode.LATERAL_ACCEL, lateralAccel, throttle);
    }

    public boolean wantsThrust() { return throttle > 0.0; }
}
