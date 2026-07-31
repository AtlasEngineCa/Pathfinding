package ca.atlasengine.pathfinding.event;

import ca.atlasengine.pathfinding.EntityNavigationController;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.result.PathStatus;
import ca.atlasengine.pathfinding.result.PathStop;

import java.util.List;

/**
 * A search landed and the controller adopted its route.
 *
 * <p>Announced when the route is taken up, not when the worker finished, so a
 * listener runs on the tick thread and sees the route the follower will walk.
 * A shed request is not a result and is announced by {@link PathShedEvent}
 * instead.</p>
 */
public final class PathComputedEvent extends EntityNavigationEvent {
    private final PathStatus status;
    private final PathStop stop;
    private final List<PathNode> nodes;

    public PathComputedEvent(EntityNavigationController controller,
                             PathStatus status, PathStop stop,
                             List<PathNode> nodes) {
        super(controller);
        if (status == null || stop == null || nodes == null) {
            throw new IllegalArgumentException("path result fields");
        }
        this.status = status;
        this.stop = stop;
        this.nodes = List.copyOf(nodes);
    }

    /** What the search produced. */
    public PathStatus status() {
        return status;
    }

    /** Why the search ended, which separates unreachable from out-of-budget. */
    public PathStop stop() {
        return stop;
    }

    /** The adopted route. Immutable. */
    public List<PathNode> nodes() {
        return nodes;
    }
}
