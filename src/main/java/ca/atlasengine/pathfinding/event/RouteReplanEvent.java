package ca.atlasengine.pathfinding.event;

import ca.atlasengine.pathfinding.EntityNavigationController;

/**
 * The controller decided to plan a new route, and why.
 *
 * <p>This announces the decision, not its result; the replacement route
 * arrives later as a {@link PathComputedEvent}, or as a {@link PathShedEvent}
 * if the system was saturated.</p>
 */
public final class RouteReplanEvent extends EntityNavigationEvent {
    /** Why a replan was requested. */
    public enum Reason {
        /** A new destination was given to the controller. */
        NEW_TARGET,
        /** A block changed near the stretch of route still to be walked. */
        BLOCK_CHANGE,
        /** The route ran out short of the target and may be extended. */
        PARTIAL_CONTINUATION,
        /** A pending search appears lost and is being replaced. */
        SEARCH_STALLED,
        /** A previous request was shed and the backoff has elapsed. */
        SHED_RETRY,
        /** The caller asked for one explicitly. */
        REQUESTED
    }

    private final Reason reason;

    public RouteReplanEvent(EntityNavigationController controller,
                            Reason reason) {
        super(controller);
        if (reason == null) throw new IllegalArgumentException("reason");
        this.reason = reason;
    }

    public Reason reason() {
        return reason;
    }
}
