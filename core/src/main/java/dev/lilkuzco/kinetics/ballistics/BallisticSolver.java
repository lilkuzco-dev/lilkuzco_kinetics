package dev.lilkuzco.kinetics.ballistics;

import dev.lilkuzco.kinetics.aero.Aerodynamics;
import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.env.Environment;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.profile.Profile;

/**
 * Drag-aware ballistics: angle for range, time of flight, apex (Section 3).
 *
 * <p>With drag there is no closed form. The vacuum solution
 * ({@code R = v^2 sin(2*theta)/g}) is wrong by a wide margin for anything fast enough to
 * matter, and its answer is always optimistic - it will tell a gunner a shot is in range that
 * physically is not. So this integrates the trajectory forward and searches for the elevation
 * that lands on the target.
 *
 * <p>It uses the same {@link Aerodynamics} and atmosphere the full simulation uses, but its own
 * lightweight point-mass loop with no phase machine, no guidance and no collision. That is what
 * a fire-control predictor should be: the same physics, run cheaply, thousands of times. The
 * test battery then flies the real integrator down the solved angle and requires the two to
 * agree within 2%, which only means something because the harnesses are separate.
 *
 * <p>Range against elevation rises to a peak and falls again - two elevations reach most
 * targets, the flat direct-fire arc and the high mortar arc. Both are returned, because they
 * are tactically different: the low arc arrives sooner, the high arc clears terrain.
 */
public final class BallisticSolver {

    private final Constants k;
    private final double dt;
    private final int maxSteps;

    public BallisticSolver(Constants k) {
        this.k = k;
        // Solve at the substep rate rather than the tick rate: the predictor should be at
        // least as accurate as the thing it is predicting.
        this.dt = k.d("world.tick_seconds") / k.i("limits.default_substeps");
        this.maxSteps = 200_000;
    }

    /** The outcome of one integrated shot. */
    public record Shot(double range, double timeOfFlight, double apexAltitude,
                       Vec3 impactPoint, Vec3 impactVelocity, boolean landed) {}

    /** A firing solution. */
    public record Solution(double elevationDeg, Shot shot, boolean converged, String note) {}

    /**
     * Integrate a shot.
     *
     * @param heading      unit horizontal direction of fire
     * @param elevationDeg elevation above horizontal
     * @param targetY      world y at which the shot is considered landed
     * @param wind         constant wind vector to fly through, or {@link Vec3#ZERO}
     */
    public Shot simulate(Profile profile, Environment env, Vec3 origin, double muzzleSpeed,
                         Vec3 heading, double elevationDeg, double targetY, Vec3 wind) {
        Aerodynamics aero = profile.airframe().aerodynamics(k);
        double mass = profile.payloadDryMass();
        double refArea = profile.airframe().referenceArea();
        double gravity = env.gravity();

        Vec3 h = heading.normalized();
        double elevRad = Math.toRadians(elevationDeg);
        Vec3 velocity = h.scale(muzzleSpeed * Math.cos(elevRad))
                .add(Vec3.UP.scale(muzzleSpeed * Math.sin(elevRad)));
        Vec3 position = origin;

        double apex = env.altitudeOf(origin.y());
        double t = 0.0;

        for (int i = 0; i < maxSteps; i++) {
            double rho = env.densityAt(position.y());
            Vec3 airspeed = velocity.sub(wind);
            double mach = env.machAt(airspeed.length(), position.y());

            // A shell has no lifting surface, so the aero call returns drag only and the
            // orientation passed in is simply the flight direction.
            Aerodynamics.AeroResult res = aero.compute(airspeed, airspeed.normalized(),
                    rho, mach, refArea, 0.0);

            Vec3 accel = res.dragForce().scale(1.0 / mass).add(new Vec3(0, -gravity, 0));
            velocity = velocity.add(accel.scale(dt));      // RA1 ordering, same as the sim
            Vec3 next = position.add(velocity.scale(dt));
            t += dt;

            double altitude = env.altitudeOf(next.y());
            if (altitude > apex) apex = altitude;

            if (next.y() <= targetY && velocity.y() < 0.0) {
                // Interpolate to the crossing so the range is not quantised to the step.
                double frac = (position.y() - targetY) / Math.max(position.y() - next.y(), 1e-12);
                Vec3 impact = position.add(next.sub(position).scale(frac));
                double range = horizontalDistance(origin, impact);
                return new Shot(range, t - dt * (1.0 - frac), apex, impact, velocity, true);
            }
            position = next;

            if (!position.isFinite()) break;
        }
        return new Shot(horizontalDistance(origin, position), t, apex, position, velocity, false);
    }

    /**
     * Find the elevation that puts a shot on a target at {@code targetRange}.
     *
     * @param highArc true for the mortar solution, false for the flat direct-fire one
     */
    public Solution solveForRange(Profile profile, Environment env, Vec3 origin,
                                  double muzzleSpeed, Vec3 heading, double targetRange,
                                  double targetY, Vec3 wind, boolean highArc) {

        double peakElevation = findMaxRangeElevation(profile, env, origin, muzzleSpeed,
                heading, targetY, wind);
        Shot peakShot = simulate(profile, env, origin, muzzleSpeed, heading,
                peakElevation, targetY, wind);

        if (peakShot.range() < targetRange) {
            return new Solution(peakElevation, peakShot, false, String.format(
                    "out of range: the best elevation (%.2f deg) reaches %.2f m, target is at "
                    + "%.2f m. Drag has already been accounted for - the vacuum equation would "
                    + "have said this was possible.", peakElevation, peakShot.range(), targetRange));
        }

        // Range is monotonic on each side of the peak, so bisect within one branch.
        double lo = highArc ? peakElevation : 0.0;
        double hi = highArc ? 89.5 : peakElevation;

        Shot best = null;
        double bestElevation = lo;
        for (int i = 0; i < 60; i++) {
            double mid = 0.5 * (lo + hi);
            Shot shot = simulate(profile, env, origin, muzzleSpeed, heading, mid, targetY, wind);
            best = shot;
            bestElevation = mid;
            boolean tooFar = shot.range() > targetRange;
            // Above the peak, range falls as elevation rises; below it, range rises.
            if (highArc == tooFar) lo = mid; else hi = mid;
            if (Math.abs(shot.range() - targetRange) < 0.01) break;
        }
        boolean converged = best != null && Math.abs(best.range() - targetRange) < 0.5;
        return new Solution(bestElevation, best, converged,
                converged ? "converged" : String.format(
                        "did not converge: closest approach %.3f m against a %.3f m target",
                        best == null ? Double.NaN : best.range(), targetRange));
    }

    /** Elevation giving maximum range. With drag this is below 45 degrees, often well below. */
    public double findMaxRangeElevation(Profile profile, Environment env, Vec3 origin,
                                        double muzzleSpeed, Vec3 heading, double targetY,
                                        Vec3 wind) {
        // Golden-section search on a unimodal curve.
        double a = 1.0;
        double b = 89.0;
        double invPhi = (Math.sqrt(5.0) - 1.0) / 2.0;
        double c = b - invPhi * (b - a);
        double d = a + invPhi * (b - a);
        double fc = simulate(profile, env, origin, muzzleSpeed, heading, c, targetY, wind).range();
        double fd = simulate(profile, env, origin, muzzleSpeed, heading, d, targetY, wind).range();

        for (int i = 0; i < 40 && (b - a) > 1e-3; i++) {
            if (fc > fd) {
                b = d; d = c; fd = fc;
                c = b - invPhi * (b - a);
                fc = simulate(profile, env, origin, muzzleSpeed, heading, c, targetY, wind).range();
            } else {
                a = c; c = d; fc = fd;
                d = a + invPhi * (b - a);
                fd = simulate(profile, env, origin, muzzleSpeed, heading, d, targetY, wind).range();
            }
        }
        return 0.5 * (a + b);
    }

    /**
     * Fire at a world coordinate: works out the heading and the elevation together.
     *
     * @return the solution, or one flagged unconverged if the point cannot be reached
     */
    public Solution fireAt(Profile profile, Environment env, Vec3 origin, Vec3 targetPoint,
                           double muzzleSpeed, Vec3 wind, boolean highArc) {
        Vec3 delta = targetPoint.sub(origin);
        Vec3 heading = new Vec3(delta.x(), 0.0, delta.z());
        if (heading.lengthSq() < 1e-12) heading = new Vec3(1, 0, 0);
        double range = heading.length();
        return solveForRange(profile, env, origin, muzzleSpeed, heading.normalized(),
                range, targetPoint.y(), wind, highArc);
    }

    private static double horizontalDistance(Vec3 a, Vec3 b) {
        double dx = b.x() - a.x();
        double dz = b.z() - a.z();
        return Math.sqrt(dx * dx + dz * dz);
    }
}
