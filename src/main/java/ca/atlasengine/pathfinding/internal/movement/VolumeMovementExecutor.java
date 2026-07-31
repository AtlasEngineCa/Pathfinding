package ca.atlasengine.pathfinding.internal.movement;

import net.minestom.server.collision.CollisionUtils;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;

/**
 * Three-dimensional swimming and flying movement.
 */
public final class VolumeMovementExecutor implements MovementExecutor {
    @Override
    public void move(MovementContext context, Point destination) {
        Entity entity = context.entity();
        Point position = entity.getPosition();
        double dx = destination.x() - position.x();
        double dy = destination.y() - position.y();
        double dz = destination.z() - position.z();
        double distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (distance < ARRIVED_EPSILON) return;
        double amount = Math.min(context.movementPerTick(), distance);
        Vec movement = new Vec(dx / distance * amount,
                dy / distance * amount, dz / distance * amount);
        var initialProbe = CollisionUtils.handlePhysics(entity, movement);
        if (initialProbe.collisionX() || initialProbe.collisionZ()) {
            double freeX = initialProbe.collisionX() ? 0 : movement.x();
            double freeZ = initialProbe.collisionZ() ? 0 : movement.z();
            double adjustedY = movement.y();
            if (Math.hypot(freeX, freeZ) < 1.0e-9
                    && Math.abs(dy) > 1.0e-12) {
                // A normalized 3D vector can approach the exact top/bottom
                // edge of an aperture asymptotically while every horizontal
                // component is blocked. Finish a small amount of vertical
                // clearance first, then resume on the following tick.
                adjustedY = Math.copySign(Math.min(
                        context.movementPerTick(),
                        Math.max(Math.abs(dy), 0.05)), dy);
            }
            movement = new Vec(freeX, adjustedY, freeZ);
        }
        context.face(dx, dy, dz, true);
        MovementExecutor.applyDisplacement(context, movement);
    }
}
