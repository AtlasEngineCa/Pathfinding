package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.MovementExecutionMode;
import net.minestom.server.collision.CollisionUtils;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;

/**
 * Turns a single waypoint into entity motion for one navigation mode.
 */
public interface MovementExecutor {
    /**
     * Distance below which a remaining offset is treated as already arrived,
     * so normalizing by it cannot divide by an effective zero.
     */
    double ARRIVED_EPSILON = 1.0e-8;

    void move(MovementContext context, Point destination);

    /**
     * Applies a resolved per-tick displacement under either execution mode.
     */
    static void applyDisplacement(MovementContext context, Vec movement) {
        Entity entity = context.entity();
        if (context.executionMode() == MovementExecutionMode.PHYSICS_VELOCITY) {
            entity.setVelocity(movement.mul(
                    MovementContext.VELOCITY_PER_BLOCK_PER_TICK));
            return;
        }
        var physics = CollisionUtils.handlePhysics(entity, movement);
        entity.refreshPosition(physics.newPosition());
        // This controller supplies move-control displacement directly. Clear
        // residual gravity/drag velocity so it cannot pull a swimmer or flyer
        // away from the path between controller ticks.
        entity.setVelocity(Vec.ZERO);
    }
}
