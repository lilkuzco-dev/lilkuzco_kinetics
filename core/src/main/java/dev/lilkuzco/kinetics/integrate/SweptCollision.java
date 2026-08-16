package dev.lilkuzco.kinetics.integrate;

import dev.lilkuzco.kinetics.env.WorldProbe;
import dev.lilkuzco.kinetics.math.Vec3;

/**
 * Swept collision by voxel traversal (Amanatides and Woo's grid-marching algorithm).
 *
 * <p>Invariant I1 says a body may not tunnel through a block thinner than its per-substep
 * displacement. The usual approach - sample a few points along the path - only makes tunnelling
 * <em>unlikely</em>, and the failure mode is a fast body silently passing through a wall
 * roughly never until someone builds a one-block roof at the wrong height.
 *
 * <p>This marches the segment cell by cell and visits <b>every</b> voxel the path crosses, in
 * order, no matter how long the segment is. Tunnelling is therefore impossible rather than
 * improbable, and the substep-displacement budget in {@link Integrator} becomes a matter of
 * integration accuracy alone rather than a collision-safety requirement.
 *
 * <p>The starting cell is deliberately not tested. A body already occupying a block - sitting
 * in a launch silo, or resting on the ground - has not just collided with it, and testing the
 * origin would trap it permanently.
 */
public final class SweptCollision {

    private SweptCollision() {}

    /** Where a swept path first entered a solid block. */
    public record Hit(Vec3 point, Vec3 normal, double distance,
                      int blockX, int blockY, int blockZ) {}

    /**
     * March from {@code from} to {@code to}, returning the first solid block entered.
     *
     * @return the hit, or null if the whole segment is clear
     */
    public static Hit cast(WorldProbe world, Vec3 from, Vec3 to) {
        Vec3 delta = to.sub(from);
        double length = delta.length();
        if (length < 1e-12) return null;
        Vec3 dir = delta.scale(1.0 / length);

        int x = WorldProbe.floor(from.x());
        int y = WorldProbe.floor(from.y());
        int z = WorldProbe.floor(from.z());

        int stepX = signum(dir.x());
        int stepY = signum(dir.y());
        int stepZ = signum(dir.z());

        double tMaxX = firstBoundary(from.x(), dir.x(), x, stepX);
        double tMaxY = firstBoundary(from.y(), dir.y(), y, stepY);
        double tMaxZ = firstBoundary(from.z(), dir.z(), z, stepZ);

        double tDeltaX = stepX == 0 ? Double.POSITIVE_INFINITY : Math.abs(1.0 / dir.x());
        double tDeltaY = stepY == 0 ? Double.POSITIVE_INFINITY : Math.abs(1.0 / dir.y());
        double tDeltaZ = stepZ == 0 ? Double.POSITIVE_INFINITY : Math.abs(1.0 / dir.z());

        // Bound the walk. A segment of length L crosses at most L+1 boundaries per axis, so
        // this cannot terminate early on a legitimate path - it only stops a degenerate one
        // (a NaN direction that slipped through) from spinning forever.
        int maxSteps = (int) (Math.ceil(length) * 3) + 8;

        for (int i = 0; i < maxSteps; i++) {
            double t;
            Vec3 normal;
            if (tMaxX <= tMaxY && tMaxX <= tMaxZ) {
                t = tMaxX;
                if (t > length) return null;
                x += stepX;
                tMaxX += tDeltaX;
                normal = new Vec3(-stepX, 0, 0);
            } else if (tMaxY <= tMaxZ) {
                t = tMaxY;
                if (t > length) return null;
                y += stepY;
                tMaxY += tDeltaY;
                normal = new Vec3(0, -stepY, 0);
            } else {
                t = tMaxZ;
                if (t > length) return null;
                z += stepZ;
                tMaxZ += tDeltaZ;
                normal = new Vec3(0, 0, -stepZ);
            }

            if (world.isSolid(x, y, z)) {
                return new Hit(from.add(dir.scale(t)), normal, t, x, y, z);
            }
        }
        return null;
    }

    private static int signum(double v) {
        if (v > 0.0) return 1;
        if (v < 0.0) return -1;
        return 0;
    }

    /** Distance along the ray to the first cell boundary on this axis. */
    private static double firstBoundary(double origin, double dir, int cell, int step) {
        if (step == 0) return Double.POSITIVE_INFINITY;
        double boundary = step > 0 ? cell + 1.0 : cell;
        return (boundary - origin) / dir;
    }
}
