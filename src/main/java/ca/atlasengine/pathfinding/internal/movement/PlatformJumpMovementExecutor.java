package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.MovementExecutionMode;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;
import ca.atlasengine.pathfinding.internal.JumpArc;
import net.minestom.server.collision.CollisionUtils;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;
import net.minestom.server.instance.Instance;

/**
 * Executes an atomic platform jump. Vertical launch velocity is written
 * once; subsequent ticks only steer horizontally while Minestom applies
 * gravity, drag, and collision.
 */
public final class PlatformJumpMovementExecutor implements MovementExecutor {
    /** Horizontal distance from the takeoff anchor treated as arrival. */
    private static final double ALIGNMENT_TOLERANCE = 1.0e-6;
    /** Ticks granted beyond the distance-derived convergence estimate. */
    private static final int ALIGNMENT_TICK_SLACK = 4;
    /** Ceiling on convergence before the route is replanned instead. */
    private static final int ALIGNMENT_TICK_LIMIT = 40;

    @Override
    public void move(MovementContext context, Point destination) {
        Entity entity = context.entity();
        Point position = entity.getPosition();
        double dx = destination.x() - position.x();
        double dz = destination.z() - position.z();
        double horizontal = Math.hypot(dx, dz);
        Vec velocity = entity.getVelocity();
        if (!context.platformJumpStarted()) {
            if (!(entity.isOnGround() || context.hasSupportBelow())) return;
            if (!convergedOnTakeoff(context, position, velocity)) return;
            PlatformJumpCapabilities jump =
                    context.profile().groundCapabilities().platformJump();
            // The search cleared the landing footprint in its opened states,
            // so opening it is the first act of the jump rather than
            // something that happens on arrival. Every other cell of the arc
            // is swept exactly as it stands, on both sides.
            BlockManipulator.openLanding(entity.getInstance(),
                    context.profile().mobProfile(), destination,
                    entity.getBoundingBox());
            if (!liveJumpArcClear(context, position, destination, jump)) {
                refuse(context, velocity);
                return;
            }
            double rise = destination.y() - position.y();
            double gravity = entity.getAerodynamics().gravity();
            double drag = entity.getAerodynamics().verticalAirResistance();
            double verticalPerTick = JumpBallistics.launchVerticalVelocity(
                    rise, jump.apexClearance(), gravity, drag);
            int flightTicks = JumpBallistics.predictedFlightTicks(
                    verticalPerTick, rise, gravity, drag);
            // An arc the entity's own aerodynamics cannot fly ends in the
            // gap. Both the short launch and the one that never descends are
            // invisible to the purely geometric arc test above.
            if (flightTicks <= 0 || !JumpBallistics.reachesApex(
                    verticalPerTick, rise, jump.apexClearance(),
                    gravity, drag)) {
                refuse(context, velocity);
                return;
            }
            context.startPlatformJump(horizontal
                    / Math.max(2, flightTicks - 1));
            velocity = velocity.withY(verticalPerTick
                    * MovementContext.VELOCITY_PER_BLOCK_PER_TICK);
        }
        double perTick = Math.min(context.platformJumpSpeed(), horizontal);
        double scale = MovementContext.VELOCITY_PER_BLOCK_PER_TICK;
        Vec steered = horizontal < 1.0e-9
                ? new Vec(0, velocity.y(), 0)
                : new Vec(dx / horizontal * perTick * scale,
                velocity.y(), dz / horizontal * perTick * scale);
        context.face(dx, destination.y() - position.y(), dz, false);
        entity.setVelocity(steered);
    }

    /**
     * The search cleared one arc: the one leaving the planned takeoff
     * anchor. A waypoint counts as reached up to half a block away from it,
     * so launching from wherever the entity happens to be standing sweeps a
     * different arc, and a refusal there is unrecoverable because the
     * recomputation it triggers re-emits the identical edge. Converging on
     * the anchor first is what makes the follower's test the planner's test.
     */
    private static boolean convergedOnTakeoff(
            MovementContext context, Point position, Vec velocity) {
        PathNode takeoff = context.previousNode();
        if (takeoff == null) return true;
        double dx = takeoff.x() - position.x();
        double dz = takeoff.z() - position.z();
        double distance = Math.hypot(dx, dz);
        if (distance <= ALIGNMENT_TOLERANCE) return true;
        if (context.countPlatformJumpAlignment()
                > alignmentTickLimit(context, distance)) {
            refuse(context, velocity);
            return false;
        }
        // One tick of velocity is applied before drag, so the closing step
        // lands on the anchor rather than short of it.
        double step = Math.min(context.movementPerTick(), distance);
        Vec movement = new Vec(
                dx / distance * step, 0, dz / distance * step);
        context.face(dx, 0, dz, false);
        Entity entity = context.entity();
        if (context.executionMode()
                == MovementExecutionMode.PHYSICS_VELOCITY) {
            double scale = MovementContext.VELOCITY_PER_BLOCK_PER_TICK;
            entity.setVelocity(new Vec(movement.x() * scale,
                    velocity.y() < 0 ? 0 : velocity.y(),
                    movement.z() * scale));
        } else {
            entity.refreshPosition(CollisionUtils.handlePhysics(
                    entity, movement).newPosition());
        }
        return false;
    }

    private static int alignmentTickLimit(
            MovementContext context, double distance) {
        return Math.min(ALIGNMENT_TICK_LIMIT, ALIGNMENT_TICK_SLACK
                + (int) Math.ceil(distance / context.movementPerTick()));
    }

    /**
     * Retaining the approach velocity would coast the entity off the takeoff
     * and into the gap it just declined to jump.
     */
    private static void refuse(MovementContext context, Vec velocity) {
        context.entity().setVelocity(new Vec(0, velocity.y(), 0));
        context.requestRecompute();
    }

    private static boolean liveJumpArcClear(
            MovementContext context, Point from, Point to,
            PlatformJumpCapabilities jump) {
        Entity entity = context.entity();
        Instance instance = entity.getInstance();
        if (instance == null) return false;
        return JumpArc.clear(instance, entity.getChunk(),
                entity.getBoundingBox(), from, to, jump.apexClearance());
    }
}
