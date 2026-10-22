package ca.atlasengine.pathfinding.internal;

import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;

/**
 * Static overlap tests between an entity box and block collision shapes.
 * The planner and the follower both judge a resting position here, so the two
 * can never disagree about whether a box fits.
 */
public final class BoxCollision {
    private BoxCollision() {
    }

    /** Whether any block shape stands inside {@code box} at {@code position}. */
    public static boolean intersects(Block.Getter blocks, BoundingBox box,
                                     Point position) {
        return intersects(blocks, box, position, false);
    }

    /**
     * Whether a climber's box fits at {@code position}. Climbable blocks are
     * held rather than collided with, so they never obstruct.
     */
    public static boolean climbClear(Block.Getter blocks, BoundingBox box,
                                     Point position) {
        return !intersects(blocks, box, position, true);
    }

    private static boolean intersects(Block.Getter blocks, BoundingBox box,
                                      Point position, boolean skipClimbables) {
        var cells = box.getBlocks(position);
        while (cells.hasNext()) {
            var cell = cells.next();
            int x = cell.blockX();
            int y = cell.blockY();
            int z = cell.blockZ();
            Block block = blocks.getBlock(x, y, z, Block.Getter.Condition.TYPE);
            if (skipClimbables && BlockTraversalData.isClimbable(block)) continue;
            if (block.collisionShape().intersectBox(
                    position.sub(new Vec(x, y, z)), box)) {
                return true;
            }
        }
        return false;
    }
}
