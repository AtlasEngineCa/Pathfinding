package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.result.PathStatus;

import java.util.List;

/**
 * Graph contract driving {@link PathSearch}. One evaluator owns the node map,
 * the terrain classification, and the movement model of a single search.
 */
abstract class NodeEvaluator {
    abstract SearchNode getStart();

    abstract List<SearchNode> getNeighbors(SearchNode current);

    /** Neighbour cells inspected by the most recent expansion. */
    abstract int lastExaminedNeighbors();

    abstract float edgeCost(SearchNode from, SearchNode to);

    abstract boolean exceedsPathLength(
            SearchNode start, SearchNode current, double maxPathLength);

    abstract PathResult result(
            PathStatus status, SearchNode end, int visited, int examined);

    boolean relaxable(SearchNode neighbor) {
        return true;
    }

    float costMalus(SearchNode neighbor) {
        return (float) neighbor.malus;
    }
}
