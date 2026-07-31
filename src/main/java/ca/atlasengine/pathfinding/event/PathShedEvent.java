package ca.atlasengine.pathfinding.event;

import ca.atlasengine.pathfinding.EntityNavigationController;

/**
 * A replan was shed by backpressure rather than searched.
 *
 * <p>Routine when the system is saturated: the entity keeps the route it
 * already has and retries after the configured backoff. A sustained rate here
 * means the worker pool is undersized, not that navigation failed.</p>
 */
public final class PathShedEvent extends EntityNavigationEvent {
    private final int retryTick;

    public PathShedEvent(EntityNavigationController controller, int retryTick) {
        super(controller);
        this.retryTick = retryTick;
    }

    /** The controller tick at which this entity may submit again. */
    public int retryTick() {
        return retryTick;
    }
}
