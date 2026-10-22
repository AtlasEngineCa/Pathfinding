package ca.atlasengine.pathfinding.internal;

import ca.atlasengine.pathfinding.result.PathNode;
import net.minestom.server.coordinate.Point;

import java.util.List;

/** Polyline lengths of a route's waypoints. */
public final class RouteLength {
    private RouteLength() {
    }

    /** Length walked from the first waypoint to the last. */
    public static double of(List<PathNode> nodes) {
        return nodes.isEmpty() ? 0
                : from(nodes.getFirst().asVec(), nodes, 1);
    }

    /**
     * Length walked from {@code start} through every waypoint from
     * {@code fromIndex} onward.
     */
    public static double from(Point start, List<PathNode> nodes, int fromIndex) {
        double length = 0;
        Point cursor = start;
        for (int index = fromIndex; index < nodes.size(); index++) {
            Point next = nodes.get(index).asVec();
            length += cursor.distance(next);
            cursor = next;
        }
        return length;
    }
}
