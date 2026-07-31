package ca.atlasengine.pathfinding.adaptive;

import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.result.PathNodeCost;
import ca.atlasengine.pathfinding.result.PathStatus;

import java.util.ArrayList;
import java.util.List;

/** A shared route composed from already validated directed plan segments. */
public final class SharedNavigationRoute<K> {
    private final K source;
    private final K target;
    private final List<NavigationPlan> segments;
    private final double cost;
    private volatile NavigationPlan combined;

    SharedNavigationRoute(
            K source, K target, List<NavigationPlan> segments, double cost) {
        this.source = source;
        this.target = target;
        this.segments = List.copyOf(segments);
        this.cost = cost;
    }

    public K source() {
        return source;
    }

    public K target() {
        return target;
    }

    public List<NavigationPlan> segments() {
        return segments;
    }

    public double cost() {
        return cost;
    }

    /**
     * Materializes and then retains one follower-ready immutable plan. Its
     * costs are those each segment's own search charged, so a composition
     * spanning segments validated by different searches reports each of them.
     */
    public NavigationPlan plan() {
        NavigationPlan existing = combined;
        if (existing != null) return existing;
        if (segments.isEmpty()) {
            throw new IllegalStateException("target node has no movement plan");
        }
        NavigationPlan first = segments.getFirst();
        NavigationPlan last = segments.getLast();
        List<PathNode> nodes = new ArrayList<>();
        List<PathNodeCost> costs = new ArrayList<>();
        boolean priced = true;
        int visited = 0;
        int examined = 0;
        for (int segmentIndex = 0;
             segmentIndex < segments.size(); segmentIndex++) {
            NavigationPlan segment = segments.get(segmentIndex);
            boolean terminal = segmentIndex == segments.size() - 1;
            if (!segment.boundingBox().equals(first.boundingBox())
                    || !segment.profile().equals(first.profile())
                    || !segment.usable()
                    || (!terminal && segment.status() != PathStatus.FOUND)
                    || segment.nodes().isEmpty()) {
                throw new IllegalStateException("incompatible route segments");
            }
            if (!nodes.isEmpty()
                    && !nodes.getLast().equals(segment.nodes().getFirst())) {
                throw new IllegalStateException("disconnected route segments");
            }
            int start = nodes.isEmpty() ? 0 : 1;
            nodes.addAll(segment.nodes().subList(start, segment.nodes().size()));
            // One unpriced segment makes the whole composition unpriced, so a
            // caller never reads a route that prices only part of itself.
            priced &= !segment.nodeCosts().isEmpty();
            if (priced) {
                costs.addAll(segment.nodeCosts().subList(
                        start, segment.nodeCosts().size()));
            }
            visited += segment.visitedNodes();
            examined += segment.examinedNeighbors();
        }
        NavigationPlan materialized = new NavigationPlan(
                first.start(), last.target(), first.boundingBox(),
                first.profile(), last.status(), nodes, visited, examined,
                priced ? costs : List.of());
        combined = materialized;
        return materialized;
    }
}
