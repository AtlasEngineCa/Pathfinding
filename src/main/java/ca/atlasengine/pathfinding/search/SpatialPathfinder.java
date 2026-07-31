package ca.atlasengine.pathfinding.search;

import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.internal.search.FlyNodeEvaluator;
import ca.atlasengine.pathfinding.internal.search.PathSearch;
import ca.atlasengine.pathfinding.internal.search.SpatialNodeEvaluator;
import ca.atlasengine.pathfinding.internal.search.SwimNodeEvaluator;
import net.minestom.server.coordinate.Point;

import java.util.List;
import static ca.atlasengine.pathfinding.internal.Points.finite;

/**
 * Weighted A* volume search for flying and swimming movement.
 */
public final class SpatialPathfinder {
    public PathResult findPath(NavigationRequest request, SearchControl control) {
        return findPathToAny(request, List.of(request.target()), control);
    }

    public PathResult findPathToAny(
            NavigationRequest request, List<? extends Point> destinations,
            SearchControl control) {
        NavigationMode type = request.profile().mode();
        if ((type != NavigationMode.FLYING && type != NavigationMode.WATER)
                || destinations == null || destinations.isEmpty()
                || destinations.stream().anyMatch(destination ->
                destination == null || !finite(destination))) {
            return PathResult.INVALID_REQUEST;
        }
        int maxVisited = (int) (Math.floor(
                request.nodeSearchRange() * 16)
                * request.maxVisitedMultiplier());
        SpatialNodeEvaluator evaluator = type == NavigationMode.WATER
                ? new SwimNodeEvaluator(request, control)
                : new FlyNodeEvaluator(request, control);
        return PathSearch.find(evaluator, destinations,
                request.maxPathLength(), request.reachRange(), maxVisited,
                control);
    }

}
