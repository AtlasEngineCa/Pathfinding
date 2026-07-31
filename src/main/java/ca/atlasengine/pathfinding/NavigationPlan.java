package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.result.PathNodeCost;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.result.PathStatus;
import ca.atlasengine.pathfinding.result.PathStop;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;

import java.util.List;
import java.util.Objects;

/**
 * Immutable, world-independent output that can be stored or given directly
 * to an entity follower.
 *
 * <p>{@code nodeCosts} is either empty or holds exactly one charged cost per
 * entry of {@code nodes}, in the same order. Composed routes therefore price
 * every cell or none of them; a plan never reports a partially priced route.
 * </p>
 */
public record NavigationPlan(
        Point start,
        Point target,
        BoundingBox boundingBox,
        NavigationProfile profile,
        PathStatus status,
        List<PathNode> nodes,
        int visitedNodes,
        int examinedNeighbors,
        List<PathNodeCost> nodeCosts,
        PathStop stop
) {
    public NavigationPlan {
        if (start == null || target == null || boundingBox == null
                || profile == null || status == null || nodes == null
                || nodes.stream().anyMatch(Objects::isNull)
                || nodeCosts == null
                || nodeCosts.stream().anyMatch(Objects::isNull)
                || visitedNodes < 0 || examinedNeighbors < 0
                || (!nodeCosts.isEmpty() && nodeCosts.size() != nodes.size())) {
            throw new IllegalArgumentException("invalid navigation plan");
        }
        start = new Vec(start.x(), start.y(), start.z());
        target = new Vec(target.x(), target.y(), target.z());
        nodes = List.copyOf(nodes);
        nodeCosts = List.copyOf(nodeCosts);
    }

    /** An unpriced plan, for callers that never ran a node-graph search. */
    public NavigationPlan(
            Point start, Point target, BoundingBox boundingBox,
            NavigationProfile profile, PathStatus status,
            List<PathNode> nodes, int visitedNodes, int examinedNeighbors) {
        this(start, target, boundingBox, profile, status, nodes,
                visitedNodes, examinedNeighbors, List.of());
    }

    /** Keeps the route, its work counters, and its per-cell charged costs. */
    public NavigationPlan(Point start, Point target, BoundingBox boundingBox,
                          NavigationProfile profile, PathStatus status,
                          List<PathNode> nodes, int visitedNodes,
                          int examinedNeighbors,
                          List<PathNodeCost> nodeCosts) {
        this(start, target, boundingBox, profile, status, nodes, visitedNodes,
                examinedNeighbors, nodeCosts,
                status == PathStatus.FOUND ? PathStop.REACHED
                        : PathStop.NOT_SEARCHED);
    }

    public static NavigationPlan from(
            NavigationRequest request, PathResult result) {
        if (request == null || result == null) {
            throw new IllegalArgumentException("request and result");
        }
        return new NavigationPlan(
                request.start(), request.target(), request.boundingBox(),
                request.profile(), result.status(), result.nodes(),
                result.visitedNodes(), result.examinedNeighbors(),
                result.nodeCosts(), result.stop());
    }

    public boolean usable() {
        return (status == PathStatus.FOUND || status == PathStatus.PARTIAL)
                && !nodes.isEmpty();
    }

    /**
     * The same route answering a request whose destination differs only in
     * ways no search can observe. The start is left alone so the
     * parity gate still compares it bit for bit.
     */
    public NavigationPlan withTarget(Point newTarget) {
        return new NavigationPlan(
                start, newTarget, boundingBox, profile, status, nodes,
                visitedNodes, examinedNeighbors, nodeCosts);
    }

    /** The same route priced as one whole search charged it. */
    public NavigationPlan withNodeCosts(List<PathNodeCost> costs) {
        return new NavigationPlan(
                start, target, boundingBox, profile, status, nodes,
                visitedNodes, examinedNeighbors, costs);
    }

    /**
     * The search result this plan carries, including the per-cell costs the
     * graph search charged. It equals a fresh {@link
     * ca.atlasengine.pathfinding.search.EntityPathfinder} result for the same request.
     */
    public PathResult result() {
        return new PathResult(status, nodes, visitedNodes,
                examinedNeighbors, nodeCosts, stop);
    }
}
