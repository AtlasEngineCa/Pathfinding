package ca.atlasengine.pathfinding.result;

import ca.atlasengine.pathfinding.NavigationPlan;import java.util.List;

/**
 * An immutable search result with basic work metrics for regression testing.
 *
 * <p>{@code nodeCosts} reports what the graph search charged for entering each
 * cell of {@code nodes}. Search-local maluses are not readable from a mob
 * profile, so this is the only view of the cost a route was priced at. It is
 * empty for results that were not produced by a node-graph search.</p>
 *
 * <p>{@code stop} reports why the search ended, which {@code status} alone
 * cannot: a {@link PathStatus#PARTIAL} whose frontier emptied says the target
 * is unreachable, while one whose frontier was cut short says only that a
 * budget ran out. A result no graph search produced reads
 * {@link PathStop#NOT_SEARCHED}, except that {@link PathStatus#FOUND} always
 * reads {@link PathStop#REACHED} because the status already names the
 * reason.</p>
 */
public record PathResult(
        PathStatus status,
        List<PathNode> nodes,
        int visitedNodes,
        int examinedNeighbors,
        List<PathNodeCost> nodeCosts,
        PathStop stop
) {
    public PathResult {
        nodes = List.copyOf(nodes);
        nodeCosts = List.copyOf(nodeCosts);
        if (stop == null) stop = PathStop.NOT_SEARCHED;
    }

    /**
     * The single routeless result every shed search completes with, so
     * sustained backpressure allocates nothing.
     */
    public static final PathResult SHED =
            new PathResult(PathStatus.SHED, List.of(), 0, 0);

    /**
     * The routeless result every rejected request returns. No search ran, so
     * there is nothing to distinguish one rejection from another.
     */
    public static final PathResult INVALID_REQUEST =
            new PathResult(PathStatus.INVALID_REQUEST, List.of(), 0, 0);

    public PathResult(PathStatus status, List<PathNode> nodes,
                      int visitedNodes, int examinedNeighbors) {
        this(status, nodes, visitedNodes, examinedNeighbors, List.of());
    }

    /**
     * A result whose search reason is not recorded. Every construction site
     * outside the graph driver builds one of these, so adding the reason
     * changed no caller.
     *
     * <p>A {@link PathStatus#FOUND} result is the one case where the status
     * already names the reason, so it reads {@link PathStop#REACHED} rather
     * than {@code NOT_SEARCHED}. The two are equivalent by definition, and the
     * consistency keeps a route that is carried through a
     * {@code NavigationPlan} and read back equal to the search it came
     * from.</p>
     */
    public PathResult(PathStatus status, List<PathNode> nodes,
                      int visitedNodes, int examinedNeighbors,
                      List<PathNodeCost> nodeCosts) {
        this(status, nodes, visitedNodes, examinedNeighbors, nodeCosts,
                status == PathStatus.FOUND
                        ? PathStop.REACHED : PathStop.NOT_SEARCHED);
    }

    public boolean found() {
        return status == PathStatus.FOUND;
    }

    /** Whether a full worker queue shed this search before it ever ran. */
    public boolean shed() {
        return status == PathStatus.SHED;
    }

    /**
     * The charged cost of one graph cell of this path, or {@code null} when
     * the cell is not on it.
     */
    public PathNodeCost costAt(int graphX, int graphY, int graphZ) {
        for (PathNodeCost cost : nodeCosts) {
            if (cost.graphX() == graphX && cost.graphY() == graphY
                    && cost.graphZ() == graphZ) return cost;
        }
        return null;
    }

    public PathResult withNodeCosts(List<PathNodeCost> costs) {
        return new PathResult(
                status, nodes, visitedNodes, examinedNeighbors, costs, stop);
    }

    /** The same result, labelled with why its search ended. */
    public PathResult withStop(PathStop reason) {
        return new PathResult(status, nodes, visitedNodes, examinedNeighbors,
                nodeCosts, reason);
    }
}
