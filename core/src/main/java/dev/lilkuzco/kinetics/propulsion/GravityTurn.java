package dev.lilkuzco.kinetics.propulsion;

import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.math.Vec3;

/**
 * The ascent pitch program (RD6).
 *
 * <p>A rocket that flies straight up reaches space and falls straight back down. Orbit is not
 * an altitude, it is a horizontal speed - roughly 1355 m/s here - so the ascent has to rotate
 * the velocity vector from vertical to horizontal on the way up. That rotation is the gravity
 * turn, and it is what physically connects "launch" to "orbital insertion" rather than
 * teleporting between them.
 *
 * <p>The program is: climb vertically out of the thickest air, pitch over a few degrees, and
 * from there let the vehicle fly at near-zero angle of attack while gravity slowly bends the
 * trajectory over. Doing it that way costs almost nothing in side loads, which is the reason
 * real vehicles do it rather than steering aggressively.
 *
 * <p><b>The max-Q constraint.</b> Aerodynamic side load scales with {@code q * alpha}. Deep in
 * the atmosphere, where q peaks, even a few degrees of angle of attack is a large bending
 * moment - so the permitted AoA is scaled down as q rises, and a pitch program that demands
 * more than the airframe allows gets clamped rather than obeyed. Ignoring this is precisely how
 * a real vehicle breaks up during ascent (RB6/RD6).
 */
public final class GravityTurn {

    private final double kickAltitude;
    private final double insertionAltitude;
    private final double pitchExponent;
    private final double aoaLimitDeg;

    /**
     * @param kickAltitude      altitude at which to stop climbing vertically and pitch over, m
     * @param insertionAltitude altitude at which the vehicle should be flying horizontally, m
     * @param pitchExponent     shapes the pitch curve; below 1 pitches over early and
     *                          aggressively, above 1 holds vertical longer
     */
    public GravityTurn(Constants k, double kickAltitude, double insertionAltitude,
                       double pitchExponent) {
        this.kickAltitude = kickAltitude;
        this.insertionAltitude = insertionAltitude;
        this.pitchExponent = pitchExponent;
        this.aoaLimitDeg = k.d("limits.ascent_aoa_limit_deg");
    }

    /**
     * Standard program: kick just out of the thick air, horizontal by the reference orbit.
     *
     * <p><b>The insertion altitude is the reference orbit, not the Karman line, and the
     * difference is the difference between reaching orbit and crashing.</b> The first version
     * pitched to fully horizontal at the Karman line - 250 m - which sounds right because that
     * is where the atmosphere ends and where insertion is judged. It is not right: a vehicle
     * flying horizontally at 250 m has stopped climbing while gravity has not stopped pulling,
     * so it arcs straight back into the ground with most of its propellant still aboard. Every
     * launch flown over real terrain crashed.
     *
     * <p>Pitching over toward the reference orbit altitude instead keeps the program steep
     * through the region that matters: at 250 m it is still 87 degrees nose-up, so the vehicle
     * climbs out of the atmosphere and only lies down once it is genuinely high.
     *
     * <p>Found by cosmos, kinetics' first consumer. The v0.1.0 golden missed it because the
     * two-stage launch flew over {@code WorldProbe.empty()} - a world with no ground cannot
     * catch a vehicle flying into the ground. The golden now flies over flat terrain.
     */
    public static GravityTurn standard(Constants k) {
        double karman = k.d("atmosphere.karman_altitude_game");
        return new GravityTurn(k, karman * 0.08, k.d("orbit.reference_orbit_altitude"), 0.6);
    }

    /**
     * Commanded pitch above horizontal, degrees. 90 is straight up, 0 is horizontal.
     */
    public double pitchDegAt(double altitude) {
        if (altitude <= kickAltitude) return 90.0;
        if (altitude >= insertionAltitude) return 0.0;
        double f = (altitude - kickAltitude) / (insertionAltitude - kickAltitude);
        return 90.0 * Math.pow(1.0 - f, pitchExponent);
    }

    /**
     * The direction to point the nose, respecting the q-dependent AoA limit.
     *
     * @param downrange unit horizontal heading the vehicle is launching toward
     * @param velocity  current velocity, used to measure the angle of attack the command implies
     * @param q         current dynamic pressure, Pa
     * @param qMax      the airframe's structural limit, Pa
     */
    public Vec3 desiredDirection(double altitude, Vec3 downrange, Vec3 velocity,
                                 double q, double qMax) {
        double pitchRad = Math.toRadians(pitchDegAt(altitude));
        Vec3 horizontal = downrange.lengthSq() < 1e-12
                ? new Vec3(1, 0, 0) : downrange.normalized();
        Vec3 commanded = horizontal.scale(Math.cos(pitchRad))
                .add(Vec3.UP.scale(Math.sin(pitchRad)))
                .normalized();

        double speed = velocity.length();
        if (speed < 1e-6 || q <= 0.0) return commanded;

        // The constraint is on the PRODUCT q*alpha, so the permitted angle is
        // (q_max * alpha_limit) / q - tight at max-Q, and correspondingly loose when there is
        // barely any air to push against.
        //
        // The first version clamped this with min(1, q_max/q), which meant the allowance never
        // rose above the base limit no matter how little dynamic pressure there was. On the pad
        // that is fatal: a vehicle at 2 m/s has no meaningful angle of attack at all, but the
        // clamp still pinned its nose to within 8 degrees of its velocity vector - and its
        // velocity vector, one substep after ignition, points DOWN, because gravity acted before
        // thrust built. The rocket then flew its own thrust into the ground. Found by cosmos.
        double allowedAoa = qMax <= 0.0 ? 180.0
                : Math.min(180.0, aoaLimitDeg * qMax / Math.max(q, 1e-9));
        Vec3 velocityDir = velocity.scale(1.0 / speed);
        double aoaDeg = Math.toDegrees(velocityDir.angleTo(commanded));
        if (aoaDeg <= allowedAoa) return commanded;

        // Clamp: rotate from the velocity vector toward the command by only the allowed angle.
        Vec3 axis = velocityDir.cross(commanded);
        if (axis.lengthSq() < 1e-18) return velocityDir;
        double allowedRad = Math.toRadians(allowedAoa);
        return velocityDir.scale(Math.cos(allowedRad))
                .add(commanded.perpendicularTo(velocityDir).normalized().scale(Math.sin(allowedRad)))
                .normalized();
    }

    /**
     * Whether the pitch program is asking for more angle of attack than the airframe can take
     * at this dynamic pressure - a structural event waiting to happen (RB6/RD6).
     */
    public boolean exceedsAoaLimit(Vec3 velocity, Vec3 commanded, double q, double qMax) {
        double speed = velocity.length();
        if (speed < 1e-6) return false;
        double allowed = qMax <= 0.0 ? 180.0
                : Math.min(180.0, aoaLimitDeg * qMax / Math.max(q, 1e-9));
        return Math.toDegrees(velocity.scale(1.0 / speed).angleTo(commanded)) > allowed;
    }

    public double kickAltitude() { return kickAltitude; }

    public double insertionAltitude() { return insertionAltitude; }
}
