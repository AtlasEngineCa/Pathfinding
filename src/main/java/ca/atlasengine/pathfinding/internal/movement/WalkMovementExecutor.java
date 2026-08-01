package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.MovementExecutionMode;
import ca.atlasengine.pathfinding.result.PathNode;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.collision.CollisionUtils;
import net.minestom.server.collision.PhysicsResult;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;

/**
 * Collision-aware ground walking, including stair/slab stepping and the
 * wall-climber impulse.
 */
public final class WalkMovementExecutor implements MovementExecutor {
    /** Rise above which a waypoint counts as an ascent rather than a step. */
    private static final double ASCENT_RISE = 0.25;
    /** Built-in climbable movement raises velocity by 0.2 blocks/tick. */
    private static final double CLIMB_IMPULSE = 4;
    /** Obstacle-ascent impulse, 0.4 blocks/tick. */
    private static final double ASCENT_IMPULSE = 8;
    /**
     * Built-in step height. A grounded entity absorbs a rise up to 0.6 without
     * leaving the ground; anything taller is an obstacle it must jump.
     */
    private static final double STEP_HEIGHT = 0.6;

    @Override
    public void move(MovementContext context, Point destination) {
        move(context, destination, context.wallClimberFallback());
    }

    public void move(MovementContext context, Point destination,
                     boolean climbOnCollision) {
        Entity entity = context.entity();
        Point position = entity.getPosition();
        double dx = destination.x() - position.x();
        double dz = destination.z() - position.z();
        double horizontal = Math.hypot(dx, dz);
        if (horizontal < ARRIVED_EPSILON) {
            if (context.executionMode()
                    == MovementExecutionMode.PHYSICS_VELOCITY) {
                Vec velocity = entity.getVelocity();
                entity.setVelocity(new Vec(0, velocity.y(), 0));
            }
            return;
        }
        double amount = Math.min(context.movementPerTick(), horizontal);
        Vec movement = new Vec(dx / horizontal * amount, 0, dz / horizontal * amount);
        context.face(dx, destination.y() - position.y(), dz, false);

        PathNode current = context.currentNode();
        boolean withinRoute = current != null;
        boolean ascends = destination.y() > position.y() + ASCENT_RISE;
        boolean stepUpNode = withinRoute
                && current.movement() == PathNode.Movement.STEP_UP;
        boolean plannedObstacleAscent =
                withinRoute && (stepUpNode || ascends);

        if (context.executionMode() == MovementExecutionMode.PHYSICS_VELOCITY) {
            Vec velocity = entity.getVelocity();
            boolean supported = entity.isOnGround() || context.hasSupportBelow();
            double velocityScale = MovementContext.VELOCITY_PER_BLOCK_PER_TICK;
            Vec nextVelocity = new Vec(
                    movement.x() * velocityScale,
                    supported && velocity.y() < 0 ? 0 : velocity.y(),
                    movement.z() * velocityScale);
            var probe = CollisionUtils.handlePhysics(entity, movement);
            boolean horizontalCollision =
                    probe.collisionX() || probe.collisionZ();
            double partialHeight = horizontalCollision
                    ? destinationPartialCollisionHeight(context, destination) : 0;
            if (climbOnCollision && horizontalCollision) {
                nextVelocity = nextVelocity.withY(CLIMB_IMPULSE);
            } else if (supported && horizontalCollision) {
                Pos stepped = steppedOverObstruction(
                        entity, movement, probe, stepSweepHeight(context));
                if (stepped != null) {
                    entity.refreshPosition(stepped);
                    nextVelocity = Vec.ZERO;
                } else if (plannedObstacleAscent
                        && (ascends || partialHeight > 0)
                        && context.claimGroundImpulse()) {
                    double verticalVelocity = partialHeight > 0
                            && partialHeight <= 0.125
                            ? (partialHeight + 0.01) * velocityScale
                            : ASCENT_IMPULSE;
                    nextVelocity = nextVelocity.withY(verticalVelocity);
                }
            }
            entity.setVelocity(nextVelocity);
            return;
        }
        var physics = CollisionUtils.handlePhysics(entity, movement);
        boolean horizontalCollision =
                physics.collisionX() || physics.collisionZ();
        boolean supported = entity.isOnGround() || context.hasSupportBelow();
        Pos stepped = horizontalCollision && supported
                ? steppedOverObstruction(entity, movement, physics,
                stepSweepHeight(context)) : null;
        entity.refreshPosition(
                stepped != null ? stepped : physics.newPosition());
        if (stepped != null) return;
        if (plannedObstacleAscent && ascends && horizontalCollision
                && entity.isOnGround()) {
            entity.setVelocity(entity.getVelocity().withY(ASCENT_IMPULSE));
        } else if (climbOnCollision && horizontalCollision) {
            // Built-in climbable movement raises velocity by 0.2 blocks/tick.
            entity.setVelocity(entity.getVelocity().withY(CLIMB_IMPULSE));
        }
    }

    private static double destinationPartialCollisionHeight(
            MovementContext context, Point destination) {
        Entity entity = context.entity();
        Instance instance = entity.getInstance();
        if (instance == null) return 0;
        Block block = instance.getBlock(
                destination.blockX(), entity.getPosition().blockY(),
                destination.blockZ(), Block.Getter.Condition.TYPE);
        double height = block.collisionShape().relativeEnd().y();
        return height > 0 && height < 1 ? height : 0;
    }

    /**
     * Where a grounded entity ends up by absorbing whatever blocks it, or null
     * when no rise within the step height gains ground over {@code blocked}.
     *
     * <p>A step is a lift, a horizontal sweep and a settle, so running
     * that sweep is the whole rule: how high the surface ahead sits decides
     * the path, and two blocks with one collision shape answer alike however
     * they are named. A rise the sweep cannot clear leaves the caller to
     * treat the obstruction as an ascent.</p>
     */
    private static double stepSweepHeight(MovementContext context) {
        double configured = context.profile().groundCapabilities().maxStepHeight();
        return configured > 1 ? configured : STEP_HEIGHT;
    }

    private static Pos steppedOverObstruction(
            Entity entity, Vec movement, PhysicsResult blocked,
            double stepHeight) {
        Instance instance = entity.getInstance();
        if (instance == null) return null;
        BoundingBox box = entity.getBoundingBox();
        Pos base = entity.getPosition();
        PhysicsResult lift = CollisionUtils.handlePhysics(instance, null, box,
                base, new Vec(0, stepHeight, 0), null, false);
        double lifted = lift.newPosition().y() - base.y();
        if (lifted <= 1.0e-6) return null;
        PhysicsResult across = CollisionUtils.handlePhysics(instance, null,
                box, lift.newPosition(), movement, null, false);
        if (squaredGround(across.newPosition(), lift.newPosition())
                <= squaredGround(blocked.newPosition(), base)) return null;
        return CollisionUtils.handlePhysics(instance, null, box,
                across.newPosition(), new Vec(0, -lifted, 0), null, false)
                .newPosition();
    }

    private static double squaredGround(Point end, Point start) {
        double dx = end.x() - start.x();
        double dz = end.z() - start.z();
        return dx * dx + dz * dz;
    }
}
