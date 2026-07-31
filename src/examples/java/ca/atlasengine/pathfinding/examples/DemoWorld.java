package ca.atlasengine.pathfinding.examples;

import net.minestom.server.coordinate.ChunkRange;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;

/**
 * The demo terrain: a flat stone plaza plus one purpose-built course per
 * navigation feature, each a lane running east from a viewpoint.
 *
 * <p>Stone fills {@code y} in {@code [0, GROUND)}, so entities stand on
 * {@link #GROUND} and a course paints its floor on {@code GROUND - 1}.</p>
 */
public final class DemoWorld {
    public static final int GROUND = 40;

    /** Loaded radius, in chunks, around the origin. */
    private static final int LOADED_CHUNKS = 4;

    private static final Block DOOR = Block.OAK_DOOR
            .withProperty("facing", "east")
            .withProperty("hinge", "left")
            .withProperty("open", "false");
    private static final Block LADDER = Block.LADDER.withProperty("facing", "west");

    public static final Pos SPAWN = new Pos(-4.5, GROUND, 0.5, -90, 0);

    /** Open plaza west of the maze, where the chased and the crowd run. */
    public static final Pos PLAZA = new Pos(-16.5, GROUND, 0.5, -90, 0);
    public static final Pos CHASE_START = new Pos(-30.5, GROUND, 0.5);

    public static final Pos MAZE_VIEW = new Pos(-4.5, GROUND, 0.5, -90, 0);
    public static final Pos MAZE_START = new Pos(2.5, GROUND, 0.5);
    public static final Pos MAZE_GOAL = new Pos(20.5, GROUND, 0.5);

    /** Lane cells -17..-15, with one unsupported column across all three. */
    public static final Pos JUMP_VIEW = new Pos(-3.5, GROUND, -15.5, -90, 0);
    public static final Pos JUMP_START = new Pos(2.5, GROUND, -14.5);
    public static final Pos JUMP_GOAL = new Pos(16.5, GROUND, -14.5);
    public static final Pos JUMP_START_PLAIN = new Pos(2.5, GROUND, -16.5);
    public static final Pos JUMP_GOAL_PLAIN = new Pos(16.5, GROUND, -16.5);

    public static final Pos DOORS_VIEW = new Pos(-3.5, GROUND, -25.5, -90, 0);
    public static final Pos DOORS_START = new Pos(2.5, GROUND, -23.5);
    public static final Pos DOORS_GOAL = new Pos(16.5, GROUND, -23.5);
    public static final Pos DOORS_START_PLAIN = new Pos(2.5, GROUND, -27.5);
    public static final Pos DOORS_GOAL_PLAIN = new Pos(16.5, GROUND, -27.5);

    public static final Pos CLIMB_VIEW = new Pos(0.5, GROUND, -36.5, -90, 0);
    public static final Pos CLIMB_START = new Pos(3.5, GROUND, -33.5);
    public static final Pos CLIMB_GOAL = new Pos(8.5, GROUND + 5, -33.5);
    public static final Pos CLIMB_START_PLAIN = new Pos(3.5, GROUND, -37.5);
    public static final Pos CLIMB_GOAL_PLAIN = new Pos(8.5, GROUND + 5, -37.5);

    public static final Pos POND_VIEW = new Pos(0.5, GROUND, 18.5, -90, 0);
    public static final Pos SWIM_START = new Pos(4.5, 37.5, 18.5);
    public static final Pos SWIM_GOAL = new Pos(18.5, 37.5, 18.5);

    public static final Pos FLY_VIEW = new Pos(-2.5, GROUND + 4, 30.5, -90, 0);
    public static final Pos FLY_START = new Pos(2.5, GROUND + 4.5, 30.5);
    public static final Pos FLY_GOAL = new Pos(18.5, GROUND + 4.5, 30.5);

    public static final Pos CUSTOM_VIEW = new Pos(-3.5, GROUND, 40.5, -90, 0);
    public static final Pos CUSTOM_START = new Pos(2.5, GROUND, 40.5);
    public static final Pos CUSTOM_GOAL = new Pos(24.5, GROUND, 40.5);

    /** Floor cells marked in gold and forbidden by a zone influence. */
    public static final int CLOSED_ROAD_MIN_X = 17;
    public static final int CLOSED_ROAD_MAX_X = 19;
    public static final int CLOSED_ROAD_MIN_Z = 39;
    public static final int CLOSED_ROAD_MAX_Z = 40;

    public static final Vec CLOSED_ROAD_FIRST =
            new Vec(CLOSED_ROAD_MIN_X, GROUND, CLOSED_ROAD_MIN_Z);
    public static final Vec CLOSED_ROAD_LAST =
            new Vec(CLOSED_ROAD_MAX_X, GROUND + 2, CLOSED_ROAD_MAX_Z);

    /** The block the custom mob's own classification prices as damaging. */
    public static final Block HAZARD = Block.MAGENTA_WOOL;

    private DemoWorld() {
    }

    public static void build(Instance instance) {
        ChunkRange.chunksInRange(0, 0, LOADED_CHUNKS,
                (x, z) -> instance.loadChunk(x, z).join());
        maze(instance);
        jumpLane(instance);
        doorCorridor(instance, -24);
        doorCorridor(instance, -28);
        climbLane(instance, -34);
        climbLane(instance, -38);
        pond(instance);
        flyLane(instance);
        customLane(instance);
    }

    /** Three staggered walls, so no route through is a straight line. */
    private static void maze(Instance instance) {
        mazeWall(instance, 6, 3, 5);
        mazeWall(instance, 11, -5, -3);
        mazeWall(instance, 16, 3, 5);
        goal(instance, 20, 0);
    }

    private static void mazeWall(Instance instance, int x,
                                 int gapMin, int gapMax) {
        for (int z = -11; z <= 11; z++) {
            if (z >= gapMin && z <= gapMax) continue;
            wall(instance, x, x, z, z, GROUND, GROUND + 2);
        }
    }

    /** A three-wide corridor cut through by one unsupported cell. */
    private static void jumpLane(Instance instance) {
        corridor(instance, 0, 17, -17, -15, 3);
        for (int z = -17; z <= -15; z++) {
            for (int y = 0; y < GROUND; y++) {
                instance.setBlock(9, y, z, Block.AIR);
            }
        }
        goal(instance, 16, -15);
        goal(instance, 16, -17);
    }

    /** A one-wide corridor with two doors in series. */
    private static void doorCorridor(Instance instance, int z) {
        corridor(instance, 0, 17, z, z, 2);
        for (int x : new int[]{7, 12}) {
            instance.setBlock(x, GROUND, z, DOOR.withProperty("half", "lower"));
            instance.setBlock(x, GROUND + 1, z, DOOR.withProperty("half", "upper"));
        }
        goal(instance, 16, z);
    }

    /** A ladder column against a wall, with a landing on top. */
    private static void climbLane(Instance instance, int z) {
        corridor(instance, 3, 8, z, z, 6);
        for (int y = GROUND; y <= GROUND + 4; y++) {
            instance.setBlock(6, y, z, LADDER);
            instance.setBlock(7, y, z, Block.STONE);
        }
        instance.setBlock(8, GROUND + 4, z, Block.EMERALD_BLOCK);
    }

    /** A sunken pond with a submerged wall the swimmer must rise over. */
    private static void pond(Instance instance) {
        for (int x = 2; x <= 20; x++) {
            for (int z = 14; z <= 22; z++) {
                for (int y = 34; y < GROUND; y++) {
                    instance.setBlock(x, y, z, Block.WATER);
                }
            }
        }
        for (int z = 14; z <= 22; z++) {
            for (int y = 34; y <= 37; y++) {
                instance.setBlock(11, y, z, Block.STONE);
            }
        }
    }

    /** A slab of wall that only a three-dimensional route clears. */
    private static void flyLane(Instance instance) {
        wall(instance, 10, 10, 26, 34, GROUND, GROUND + 6);
    }

    /**
     * A corridor carrying one of each thing the custom profile answers: a
     * block only its own classification knows, a gap only its capabilities
     * cross, and a zone only its request-scoped influence forbids.
     */
    private static void customLane(Instance instance) {
        corridor(instance, 0, 25, 39, 41, 3);
        for (int x = 5; x <= 7; x++) {
            for (int z = 40; z <= 41; z++) {
                instance.setBlock(x, GROUND - 1, z, HAZARD);
            }
        }
        for (int z = 39; z <= 41; z++) {
            for (int y = 0; y < GROUND; y++) {
                instance.setBlock(12, y, z, Block.AIR);
            }
        }
        for (int x = CLOSED_ROAD_MIN_X; x <= CLOSED_ROAD_MAX_X; x++) {
            for (int z = CLOSED_ROAD_MIN_Z; z <= CLOSED_ROAD_MAX_Z; z++) {
                instance.setBlock(x, GROUND - 1, z, Block.GOLD_BLOCK);
            }
        }
        goal(instance, 24, 40);
    }

    private static void goal(Instance instance, int x, int z) {
        instance.setBlock(x, GROUND - 1, z, Block.EMERALD_BLOCK);
    }

    /**
     * A closed corridor over the walkable cells {@code [minX, maxX]} by
     * {@code [minZ, maxZ]}, capped at both ends so the only route through it
     * is the feature the lane exists to show.
     */
    private static void corridor(Instance instance, int minX, int maxX,
                                 int minZ, int maxZ, int height) {
        int top = GROUND + height;
        wall(instance, minX, maxX, minZ - 1, minZ - 1, GROUND, top);
        wall(instance, minX, maxX, maxZ + 1, maxZ + 1, GROUND, top);
        wall(instance, minX - 1, minX - 1, minZ - 1, maxZ + 1, GROUND, top);
        wall(instance, maxX + 1, maxX + 1, minZ - 1, maxZ + 1, GROUND, top);
    }

    private static void wall(Instance instance, int minX, int maxX,
                             int minZ, int maxZ, int minY, int maxY) {
        for (int x = minX; x <= maxX; x++) {
            for (int z = minZ; z <= maxZ; z++) {
                for (int y = minY; y <= maxY; y++) {
                    instance.setBlock(x, y, z, Block.STONE);
                }
            }
        }
    }
}
