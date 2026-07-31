package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.MovementExecutionMode;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.result.PathNode;
import net.minestom.server.ServerFlag;
import net.minestom.server.collision.CollisionUtils;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;

/**
 * Live view of the navigation state a {@link MovementExecutor} needs.
 */
public interface MovementContext {
    /** Velocity units per block of displacement per tick. */
    double VELOCITY_PER_BLOCK_PER_TICK = ServerFlag.SERVER_TICKS_PER_SECOND;

    Entity entity();

    double movementPerTick();

    MovementExecutionMode executionMode();

    NavigationProfile profile();

    /** The waypoint's node, or null once the route is consumed. */
    PathNode currentNode();

    /** The node the current waypoint is travelled from, or null. */
    PathNode previousNode();

    boolean wallClimberFallback();

    /** True the first time it is called for the current waypoint. */
    boolean claimGroundImpulse();

    boolean platformJumpStarted();

    /** Ticks spent converging on this jump's takeoff, counting this one. */
    int countPlatformJumpAlignment();

    void startPlatformJump(double horizontalSpeed);

    double platformJumpSpeed();

    void requestRecompute();

    default void face(double dx, double dy, double dz, boolean pitch) {
        Entity entity = entity();
        float wantedYaw = (float) Math.toDegrees(Math.atan2(dz, dx)) - 90;
        float yaw = rotateToward(entity.getPosition().yaw(), wantedYaw, 90);
        float wantedPitch = pitch
                ? (float) -Math.toDegrees(
                        Math.atan2(dy, Math.hypot(dx, dz)))
                : entity.getPosition().pitch();
        entity.setView(yaw, wantedPitch);
    }

    /**
     * Minestom's grounded flag is the result of the preceding physics tick.
     * It can remain false for a custom wide bounding box even while a
     * downward sweep is resting on a floor. Retaining its accumulated
     * negative velocity in that state can suppress otherwise-clear
     * horizontal movement. Probe the actual collision geometry without
     * treating entities over a genuine drop as supported.
     */
    default boolean hasSupportBelow() {
        Entity entity = entity();
        var result = CollisionUtils.handlePhysics(
                entity, new Vec(0, -0.05, 0));
        return result.collisionY()
                || result.newPosition().y()
                > entity.getPosition().y() - 0.049;
    }

    private static float rotateToward(
            float current, float wanted, float maximum) {
        float delta = (wanted - current) % 360;
        if (delta < -180) delta += 360;
        if (delta >= 180) delta -= 360;
        delta = Math.max(-maximum, Math.min(maximum, delta));
        return current + delta;
    }
}
