package dev.lilkuzco.kinetics.fabric;

import dev.lilkuzco.kinetics.env.WorldProbe;
import net.minecraft.core.BlockPos;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.block.state.BlockState;

/**
 * The entire bridge between kinetics and Minecraft.
 *
 * <p>One class, one method of substance. Everything the physics library is allowed to know about
 * the world comes through here, which is what makes the whole of {@code kinetics-core} testable
 * headless and what makes invariant I10 checkable by inspection - there is no reference to an
 * entity, a damage source or a block-breaking call anywhere in this file, and no other file has
 * a route to one.
 *
 * <p>Solidity is decided by whether a block's collision shape is empty. That treats fluids,
 * grass, torches and air alike as passable, which is the right answer for a missile: it is
 * stopped by the things that would stop it.
 *
 * <p>Block lookups are cached for the duration of a tick. A guided body samples line of sight
 * every substep and a radar horizon walks hundreds of blocks per query, so the same handful of
 * chunk sections get hit over and over; caching them turns the hot path from a chunk lookup into
 * an array read. The cache is cleared at the tick boundary, so it can never serve a stale answer
 * across a block change - which would also break the purity {@link WorldProbe} requires for I7.
 */
public final class MinecraftWorldProbe implements WorldProbe {

    private final ServerLevel level;
    private final BlockPos.MutableBlockPos scratch = new BlockPos.MutableBlockPos();

    // Small direct-mapped cache keyed on the packed block position. Sized as a power of two so
    // the index is a mask rather than a modulo.
    private static final int CACHE_BITS = 12;
    private static final int CACHE_SIZE = 1 << CACHE_BITS;
    private final long[] cacheKey = new long[CACHE_SIZE];
    private final boolean[] cacheValue = new boolean[CACHE_SIZE];
    private boolean cachePrimed;

    private long queries;
    private long cacheHits;

    public MinecraftWorldProbe(ServerLevel level) {
        this.level = level;
    }

    @Override
    public boolean isSolid(int blockX, int blockY, int blockZ) {
        queries++;
        long key = BlockPos.asLong(blockX, blockY, blockZ);
        int slot = (int) ((key * 0x9E3779B97F4A7C15L) >>> (64 - CACHE_BITS));

        if (cachePrimed && cacheKey[slot] == key) {
            cacheHits++;
            return cacheValue[slot];
        }

        scratch.set(blockX, blockY, blockZ);
        // Outside the build limits there is nothing to hit. A rocket above the ceiling is in
        // clear sky, not inside the world's lid.
        boolean solid;
        if (blockY < level.getMinY() || blockY >= level.getMaxY()) {
            solid = false;
        } else {
            BlockState state = level.getBlockState(scratch);
            solid = !state.getCollisionShape(level, scratch).isEmpty();
        }

        cacheKey[slot] = key;
        cacheValue[slot] = solid;
        cachePrimed = true;
        return solid;
    }

    /**
     * Drop the cache. Called at the tick boundary so a block placed this tick is visible next
     * tick, and so the probe stays pure within any single tick as {@link WorldProbe} requires.
     */
    public void endTick() {
        if (cachePrimed) {
            java.util.Arrays.fill(cacheKey, 0L);
            cachePrimed = false;
        }
    }

    public ServerLevel level() { return level; }

    public long queries() { return queries; }

    /** Cache hit rate, for the performance report. */
    public double hitRate() {
        return queries == 0 ? 0.0 : (double) cacheHits / queries;
    }
}
