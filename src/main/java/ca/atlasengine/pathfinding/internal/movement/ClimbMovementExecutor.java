package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;

/**
 * Ladder and vine movement, plus the horizontal approach that carries an
 * entity into a climbable block.
 */
public final class ClimbMovementExecutor implements MovementExecutor {
    @Override
    public void move(MovementContext context, Point destination) {
        Entity entity = context.entity();
        Point position = entity.getPosition();
        double dx = destination.x() - position.x();
        double dy = destination.y() - position.y();
        double dz = destination.z() - position.z();
        double horizontal = Math.hypot(dx, dz);
        if (horizontal > ARRIVED_EPSILON) context.face(dx, dy, dz, false);
        double horizontalAmount = Math.min(context.movementPerTick(), horizontal);
        double moveX = horizontal > ARRIVED_EPSILON
                ? dx / horizontal * horizontalAmount : 0;
        double moveZ = horizontal > ARRIVED_EPSILON
                ? dz / horizontal * horizontalAmount : 0;
        // LivingEntity clamps downward climbable motion to -0.15 and raises
        // colliding climbers by +0.2 blocks/tick. Planned climb edges use the
        // same vertical limits without requiring an accidental wall impact.
        double moveY = dy > 0
                ? Math.min(0.2, dy) : Math.max(-0.15, dy);
        Point next = position.add(moveX, moveY, moveZ);
        if (climbMovementClear(context, next)) {
            entity.refreshPosition(next.asPos());
            entity.setVelocity(Vec.ZERO);
            return;
        }
        MovementExecutor.applyDisplacement(
                context, new Vec(moveX, moveY, moveZ));
    }

    public void moveIntoClimbable(MovementContext context, Point destination) {
        Entity entity = context.entity();
        Point position = entity.getPosition();
        double dx = destination.x() - position.x();
        double dz = destination.z() - position.z();
        double distance = Math.hypot(dx, dz);
        if (distance < ARRIVED_EPSILON) return;
        context.face(dx, destination.y() - position.y(), dz, false);
        double amount = Math.min(context.movementPerTick(), distance);
        Vec movement = new Vec(
                dx / distance * amount, 0, dz / distance * amount);
        Point next = position.add(movement);
        if (climbMovementClear(context, next)) {
            entity.refreshPosition(next.asPos());
            entity.setVelocity(Vec.ZERO);
        }
    }

    private static boolean climbMovementClear(
            MovementContext context, Point position) {
        Entity entity = context.entity();
        Instance instance = entity.getInstance();
        return instance != null && movementClear(
                instance, entity.getBoundingBox(), position);
    }

    /**
     * Whether this executor will drive a box to a position. The planner emits
     * only climb nodes that pass it, so it is the one rule both halves judge
     * a climb by and is public for tests to hold them to it.
     */
    public static boolean movementClear(
            Block.Getter blocks, BoundingBox box, Point position) {
        var iterator = box.getBlocks(position);
        while (iterator.hasNext()) {
            var mutable = iterator.next();
            Block block = blocks.getBlock(mutable.blockX(),
                    mutable.blockY(), mutable.blockZ(),
                    Block.Getter.Condition.TYPE);
            if (BlockTraversalData.isClimbable(block)) continue;
            Point blockPosition = new Vec(mutable.blockX(), mutable.blockY(),
                    mutable.blockZ());
            if (block.collisionShape().intersectBox(
                    position.sub(blockPosition), box)) {
                return false;
            }
        }
        return true;
    }
}
