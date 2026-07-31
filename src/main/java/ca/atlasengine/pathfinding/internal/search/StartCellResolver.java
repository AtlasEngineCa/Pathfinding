package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.search.EntityTraversalState;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block;

/**
 * Finds the cell a search begins from.
 *
 * <p>An entity's own position is a poor answer on its own. It may be
 * mid-fall, submerged, standing on a fluid it treats as ground, or straddling
 * a cell boundary so that the block it is nominally in is not the block it is
 * supported by. Each of those wants a different column scan, so the vertical
 * search is chosen from the entity's state first and only then run.</p>
 *
 * <p>The resolver always yields a node. A search that cannot begin anywhere
 * legal is still a search that must report a start, and refusing here would
 * turn a recoverable "no route" into a failure the caller cannot describe.</p>
 */
final class StartCellResolver {
    private final GroundNodeEvaluator evaluator;
    private final Block.Getter blocks;
    private final Point startPosition;
    private final BoundingBox box;
    private final EntityTraversalState entityState;

    StartCellResolver(GroundNodeEvaluator evaluator) {
        this.evaluator = evaluator;
        this.blocks = evaluator.blocks;
        this.startPosition = evaluator.startPosition;
        this.box = evaluator.box;
        this.entityState = evaluator.entityState;
    }

    SearchNode resolve() {
        int x = startPosition.blockX();
        int z = startPosition.blockZ();
        int y = supportedY(x, z);
        if (standable(x, y, z)) return evaluator.accepted(x, y, z);

        // The nominal column can be blocked while a corner of the box rests on
        // something legal, which is the ordinary case for a wide entity on a
        // ledge or in a doorway.
        for (int[] corner : footprintCorners()) {
            if (standable(corner[0], y, corner[1])) {
                return evaluator.accepted(corner[0], y, corner[1]);
            }
        }
        // Retain the nominal node so callers always receive a usable origin.
        return evaluator.accepted(x, y, z);
    }

    /** The Y the entity is actually supported at, by whichever rule applies. */
    private int supportedY(int x, int z) {
        int y = startPosition.blockY();
        Block current = blocks.getBlock(x, y, z, Block.Getter.Condition.TYPE);

        // Standing on a fluid surface: rise to the top of the fluid column.
        if (entityState.canStandOn(current)) {
            while (entityState.canStandOn(blocks.getBlock(
                    x, y, z, Block.Getter.Condition.TYPE))) y++;
            return y - 1;
        }
        // Swimming: rise to the surface rather than route from the seabed.
        if (evaluator.profile.canFloat() && entityState.inWater()) {
            while (GroundNodeEvaluator.isWater(blocks.getBlock(
                    x, y, z, Block.Getter.Condition.TYPE))) y++;
            return y - 1;
        }
        // On the ground: the half-block bias picks the cell the feet are in
        // rather than the one the origin rounds to.
        if (entityState.onGround()) {
            return (int) Math.floor(startPosition.y() + 0.5);
        }
        return firstSupportBelow(x, z);
    }

    /** Mid-air: fall to the first cell that would stop the entity. */
    private int firstSupportBelow(int x, int z) {
        int y = startPosition.blockY();
        int cursorY = (int) Math.floor(startPosition.y() + 1);
        while (cursorY > entityState.minBuildHeight()) {
            y = cursorY;
            cursorY--;
            Block below = blocks.getBlock(
                    x, cursorY, z, Block.Getter.Condition.TYPE);
            if (!below.air() && !BlockTraversalData.isLandPathfindable(below)) {
                break;
            }
        }
        return y;
    }

    private int[][] footprintCorners() {
        int minX = (int) Math.floor(
                startPosition.x() + box.relativeStart().x());
        int maxX = (int) Math.floor(
                startPosition.x() + box.relativeEnd().x());
        int minZ = (int) Math.floor(
                startPosition.z() + box.relativeStart().z());
        int maxZ = (int) Math.floor(
                startPosition.z() + box.relativeEnd().z());
        return new int[][]{
                {minX, minZ}, {minX, maxZ}, {maxX, minZ}, {maxX, maxZ}
        };
    }

    /**
     * OPEN is refused deliberately: a start cell with nothing underfoot would
     * let a search set out from mid-air and return a route the follower cannot
     * begin walking.
     */
    private boolean standable(int x, int y, int z) {
        TerrainType type = evaluator.type(x, y, z);
        return type != TerrainType.OPEN && evaluator.profile.malus(type) >= 0;
    }
}
