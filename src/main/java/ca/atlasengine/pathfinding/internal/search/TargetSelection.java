package ca.atlasengine.pathfinding.internal.search;

import java.util.List;

/**
 * Per-destination bookkeeping and result ranking shared by the multi-target
 * ground and volume searches. A reached destination is ranked by fewest path
 * nodes; an unreached one by smallest Manhattan distance, then fewest nodes.
 */
final class TargetSelection {
    private final int x;
    private final int y;
    private final int z;
    private float bestDistance = Float.MAX_VALUE;
    private SearchNode best;
    private boolean reached;

    TargetSelection(int x, int y, int z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    float distanceTo(SearchNode node) {
        float dx = x - node.x;
        float dy = y - node.y;
        float dz = z - node.z;
        return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    /**
     * Scores a node against this target and records it as the closest yet if
     * it is.
     *
     * <p>The two are deliberately one operation. A search that ends without
     * arriving still has to return something, and what it returns is the
     * closest node it ever scored — so the set of nodes eligible to be that
     * fallback is exactly the set the heuristic was computed for. Splitting
     * them would let the two drift, and a partial route would start depending
     * on which call sites remembered to record.</p>
     */
    float observe(SearchNode node) {
        float distance = distanceTo(node);
        track(distance, node);
        return distance;
    }

    int manhattanTo(SearchNode node) {
        return Math.abs(node.x - x) + Math.abs(node.y - y)
                + Math.abs(node.z - z);
    }

    void markReached() {
        reached = true;
    }

    SearchNode bestNode() {
        if (best == null) throw new IllegalStateException("unscored target");
        return best;
    }

    private void track(float distance, SearchNode candidate) {
        if (distance < bestDistance) {
            bestDistance = distance;
            best = candidate;
        }
    }

    /**
     * Observes a node against every target and returns the nearest distance.
     * Each target tracks the node independently, so a multi-target search that
     * falls short can still report the best approach to each destination.
     */
    static float observeAll(SearchNode node, List<TargetSelection> targets) {
        float best = Float.MAX_VALUE;
        for (TargetSelection target : targets) {
            best = Math.min(best, target.observe(node));
        }
        return best;
    }

    static SearchNode selectReached(List<TargetSelection> targets) {
        SearchNode selected = null;
        int selectedCount = Integer.MAX_VALUE;
        for (TargetSelection target : targets) {
            if (!target.reached) continue;
            int count = nodeCount(target.best);
            if (count < selectedCount) {
                selected = target.best;
                selectedCount = count;
            }
        }
        if (selected == null) throw new IllegalStateException("no reached target");
        return selected;
    }

    static SearchNode selectUnreached(List<TargetSelection> targets) {
        SearchNode selected = null;
        int selectedDistance = Integer.MAX_VALUE;
        int selectedCount = Integer.MAX_VALUE;
        for (TargetSelection target : targets) {
            int candidateDistance = target.manhattanTo(target.best);
            int candidateCount = nodeCount(target.best);
            if (candidateDistance < selectedDistance
                    || candidateDistance == selectedDistance
                    && candidateCount < selectedCount) {
                selected = target.best;
                selectedDistance = candidateDistance;
                selectedCount = candidateCount;
            }
        }
        if (selected == null) throw new IllegalStateException("no target");
        return selected;
    }

    private static int nodeCount(SearchNode node) {
        int count = 0;
        for (SearchNode cursor = node; cursor != null; cursor = cursor.previous) {
            count++;
        }
        return count;
    }
}
