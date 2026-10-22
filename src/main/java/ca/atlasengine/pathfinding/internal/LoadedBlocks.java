package ca.atlasengine.pathfinding.internal;

import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.result.PathStop;
import net.minestom.server.coordinate.CoordConversion;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;

/**
 * Block reads that treat unloaded terrain as solid.
 *
 * <p>{@link Instance#getBlock} throws for a chunk that is not loaded, and a
 * search near the edge of the loaded area reaches such chunks routinely. Read
 * through this view instead, unloaded terrain is a wall: no route enters it,
 * and a search that would have thrown ends as an ordinary partial route.</p>
 */
public final class LoadedBlocks implements Block.Getter {
    /** What every cell of an unloaded chunk reads as. */
    private static final Block UNLOADED = Block.STONE;

    private final Instance instance;
    /** The chunk last seen loaded, so a run of reads in it checks once. */
    private int loadedChunkX = Integer.MIN_VALUE;
    private int loadedChunkZ = Integer.MIN_VALUE;
    private boolean readUnloaded;

    private LoadedBlocks(Instance instance) {
        this.instance = instance;
    }

    /**
     * A view of {@code blocks} that never throws for unloaded terrain. Reads
     * still go through {@link Instance#getBlock}, so an instance overriding it
     * is honoured. The view remembers the chunk it last read, so it belongs to
     * one thread: create one per search or per tick-thread probe. Any other
     * getter is returned unchanged.
     */
    public static Block.Getter view(Block.Getter blocks) {
        return blocks instanceof Instance instance
                ? new LoadedBlocks(instance) : blocks;
    }

    /**
     * {@code result} as seen through {@code view}: an exhausted frontier only
     * proves the target unreachable when no unloaded terrain walled it in.
     */
    public static PathResult qualify(Block.Getter view, PathResult result) {
        return view instanceof LoadedBlocks loaded && loaded.readUnloaded
                && result.stop() == PathStop.FRONTIER_EXHAUSTED
                ? result.withStop(PathStop.TERRAIN_UNLOADED) : result;
    }

    @Override
    public Block getBlock(int x, int y, int z, Condition condition) {
        int chunkX = CoordConversion.globalToChunk(x);
        int chunkZ = CoordConversion.globalToChunk(z);
        if (chunkX != loadedChunkX || chunkZ != loadedChunkZ) {
            if (instance.getChunk(chunkX, chunkZ) == null) return unloaded();
            loadedChunkX = chunkX;
            loadedChunkZ = chunkZ;
        }
        try {
            return instance.getBlock(x, y, z, condition);
        } catch (NullPointerException unloadedSinceChecked) {
            // The chunk unloaded between the check and the read.
            loadedChunkX = Integer.MIN_VALUE;
            return unloaded();
        }
    }

    private Block unloaded() {
        readUnloaded = true;
        return UNLOADED;
    }
}
