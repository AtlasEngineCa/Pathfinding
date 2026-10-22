package ca.atlasengine.pathfinding.search;

import ca.atlasengine.pathfinding.internal.LoadedBlocks;
import ca.atlasengine.pathfinding.internal.search.FlyNodeEvaluator;
import ca.atlasengine.pathfinding.internal.search.PathSearch;
import ca.atlasengine.pathfinding.internal.search.SpatialNodeEvaluator;
import ca.atlasengine.pathfinding.internal.search.SwimNodeEvaluator;
import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.result.PathResult;
import net.minestom.server.coordinate.Point;

import java.util.List;

import static ca.atlasengine.pathfinding.internal.Points.allFinite;

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
                || !allFinite(destinations)) {
            return PathResult.INVALID_REQUEST;
        }
        NavigationRequest loaded =
                request.withBlocks(LoadedBlocks.view(request.blocks()));
        SpatialNodeEvaluator evaluator = type == NavigationMode.WATER
                ? new SwimNodeEvaluator(loaded, control)
                : new FlyNodeEvaluator(loaded, control);
        return LoadedBlocks.qualify(loaded.blocks(), PathSearch.find(
                evaluator, destinations, request.maxPathLength(),
                request.reachRange(), request.maxVisitedNodes(), control));
    }
}
