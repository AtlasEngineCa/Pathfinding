package ca.atlasengine.pathfinding.load;

import net.minestom.server.coordinate.ChunkRange;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;

import java.util.random.RandomGenerator;

/**
 * Deterministic obstacle field. Pillars sit on a four-block lattice so every
 * gap stays at least three blocks wide and the arena is always connected.
 */
final class LoadArena {
    static final int SURFACE_Y = 40;

    private final Instance instance;
    private final int half;

    private LoadArena(Instance instance, int half) {
        this.instance = instance;
        this.half = half;
    }

    static LoadArena build(
            Instance instance, int half, RandomGenerator random) {
        LoadArena arena = new LoadArena(instance, half);
        ChunkRange.chunksInRange(0, 0, half / 16 + 2,
                (x, z) -> instance.loadChunk(x, z).join());
        for (int x = -half; x <= half; x += 4) {
            for (int z = -half; z <= half; z += 4) {
                if (Math.abs(x) <= 8 && Math.abs(z) <= 8) continue;
                if (random.nextInt(100) >= 26) continue;
                arena.pillar(x, z);
            }
        }
        arena.pond();
        return arena;
    }

    Instance instance() {
        return instance;
    }

    int half() {
        return half;
    }

    Pos objective() {
        return new Pos(0.5, SURFACE_Y, 0.5);
    }

    Pos pondCentre() {
        return new Pos(-half + 12.5, SURFACE_Y, half - 12.5);
    }

    /** The pond and a margin around it, reserved for the amphibious family. */
    boolean inPond(double x, double z) {
        Pos centre = pondCentre();
        return Math.abs(x - centre.x()) <= 7 && Math.abs(z - centre.z()) <= 7;
    }

    /** Places a small wall so retained topology must be invalidated. */
    void editTerrain() {
        for (int x = 12; x <= 15; x++) {
            for (int y = SURFACE_Y; y <= SURFACE_Y + 1; y++) {
                instance.setBlock(x, y, 12, Block.STONE);
            }
        }
    }

    private void pillar(int x, int z) {
        instance.setBlock(x, SURFACE_Y, z, Block.STONE);
        instance.setBlock(x, SURFACE_Y + 1, z, Block.STONE);
    }

    private void pond() {
        Pos centre = pondCentre();
        for (int x = centre.blockX() - 4; x <= centre.blockX() + 4; x++) {
            for (int z = centre.blockZ() - 4; z <= centre.blockZ() + 4; z++) {
                instance.setBlock(x, SURFACE_Y - 1, z, Block.WATER);
                instance.setBlock(x, SURFACE_Y, z, Block.WATER);
            }
        }
    }
}
