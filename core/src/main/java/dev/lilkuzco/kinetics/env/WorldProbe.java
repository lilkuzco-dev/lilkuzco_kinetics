package dev.lilkuzco.kinetics.env;

import dev.lilkuzco.kinetics.math.Vec3;

/**
 * The entire surface through which kinetics is allowed to ask about the world.
 *
 * <p>Deliberately three methods. {@code kinetics-core} has no Minecraft dependency, so the
 * Fabric adapter implements this and nothing else crosses the boundary; that is what lets the
 * whole physics library and its test battery run headless, with a flat or scripted world
 * standing in for a server. It is also the reason I10 is enforceable - there is no handle
 * here through which damage could be applied even by accident.
 *
 * <p>Implementations must be pure with respect to a given world state: calling
 * {@link #isSolid} twice with the same arguments in the same tick must return the same
 * answer, or determinism (I7) is lost.
 */
public interface WorldProbe {

    /** Whether the block containing these integer coordinates blocks motion. */
    boolean isSolid(int blockX, int blockY, int blockZ);

    /** Empty world: no collision, no occlusion. The default for closed-form tests. */
    static WorldProbe empty() {
        return (x, y, z) -> false;
    }

    /** Flat ground at and below {@code groundY}. Used by ballistic and landing tests. */
    static WorldProbe flatGround(int groundY) {
        return (x, y, z) -> y <= groundY;
    }

    /** Whether the point is inside a solid block. */
    default boolean isSolidAt(Vec3 p) {
        return isSolid(floor(p.x()), floor(p.y()), floor(p.z()));
    }

    /**
     * Unobstructed line of sight between two points, sampled at {@code step} metres.
     *
     * <p>Used by the radar horizon (RF2), seeker occlusion (RC6) and terrain following (RC5).
     * The step defaults to half a block so no block face can be stepped over - the same
     * reasoning that governs substep displacement in I1.
     */
    default boolean lineOfSight(Vec3 from, Vec3 to, double step) {
        Vec3 delta = to.sub(from);
        double distance = delta.length();
        if (distance < 1e-9) return true;
        Vec3 dir = delta.scale(1.0 / distance);
        int samples = (int) Math.ceil(distance / step);
        // Skip the endpoints: a launcher inside its own silo block, or a target standing on
        // the ground, must not occlude itself.
        for (int i = 1; i < samples; i++) {
            Vec3 p = from.add(dir.scale(i * step));
            if (isSolidAt(p)) return false;
        }
        return true;
    }

    /**
     * Highest solid block at or below {@code fromY} in this column, or
     * {@code Integer.MIN_VALUE} if the column is clear. Terrain following (RC5) rides this.
     */
    default int groundHeight(double x, double z, int fromY, int searchDepth) {
        int bx = floor(x);
        int bz = floor(z);
        for (int y = fromY; y > fromY - searchDepth; y--) {
            if (isSolid(bx, y, bz)) return y;
        }
        return Integer.MIN_VALUE;
    }

    static int floor(double v) {
        int i = (int) v;
        return v < i ? i - 1 : i;
    }
}
