package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.result.PathNodeCost;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.result.PathStatus;
import ca.atlasengine.pathfinding.result.PathStop;
import ca.atlasengine.pathfinding.search.SearchControl;
import net.minestom.server.coordinate.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The single weighted-A* driver. Every movement model reaches it through a
 * {@link NodeEvaluator}.
 */
public final class PathSearch {
    /** Absolute runaway guard shared by every public search entry point. */
    static final int MAX_VISITED_NODES = 65_536;
    /** Destination pull: above one favors forward progress over broad search. */
    private static final float DESTINATION_PULL = 1.5F;

    private PathSearch() {
    }

    public static PathResult find(NodeEvaluator evaluator,
                                  List<? extends Point> destinations,
                                  double maxPathLength, int reachRange, int maxVisited,
                                  SearchControl control) {
        return new SearchRun(evaluator, destinations, maxPathLength,
                reachRange, maxVisited, control).execute();
    }

    private static final class SearchRun {
        private final NodeEvaluator evaluator;
        private final List<TargetSelection> targets;
        private final double distanceLimit;
        private final int arrivalRadius;
        private final int expansionLimit;
        private final SearchControl control;
        private final SearchFrontier frontier = new SearchFrontier();
        private final TargetSelection singleTarget;
        private final SearchNode origin;
        private int expanded;
        private int examined;
        private boolean distanceRejected;

        private SearchRun(
                NodeEvaluator evaluator, List<? extends Point> destinations,
                double distanceLimit, int arrivalRadius, int expansionLimit,
                SearchControl control) {
            this.evaluator = evaluator;
            if (destinations.size() == 1) {
                Point point = destinations.getFirst();
                this.singleTarget = new TargetSelection(
                        point.blockX(), point.blockY(), point.blockZ());
                this.targets = List.of(singleTarget);
            } else {
                List<TargetSelection> selections =
                        new ArrayList<>(destinations.size());
                for (Point point : destinations) {
                    selections.add(new TargetSelection(
                            point.blockX(), point.blockY(), point.blockZ()));
                }
                this.targets = selections;
                this.singleTarget = null;
            }
            this.distanceLimit = distanceLimit;
            this.arrivalRadius = arrivalRadius;
            this.expansionLimit = Math.min(expansionLimit, MAX_VISITED_NODES);
            this.control = control;
            this.origin = evaluator.getStart();
        }

        private PathResult execute() {
            seedFrontier();
            for (int attempts = 1;
                 attempts < expansionLimit && !frontier.isEmpty();
                 attempts++) {
                PathResult interrupted = interruptedResult();
                if (interrupted != null) return interrupted;

                SearchNode current = frontier.takeBest();
                current.settled = true;
                expanded++;
                if (markArrivals(current)) return finishFound();
                if (evaluator.exceedsPathLength(
                        origin, current, distanceLimit)) {
                    distanceRejected = true;
                    continue;
                }

                List<SearchNode> adjacent = evaluator.getNeighbors(current);
                examined += evaluator.lastExaminedNeighbors();
                interrupted = interruptedResult();
                if (interrupted != null) return interrupted;
                adjacent.forEach(next -> consider(current, next));
            }
            return priced(evaluator, PathStatus.PARTIAL,
                    selectedUnreached(), expanded, examined,
                    frontierStop(frontier.isEmpty(), distanceRejected));
        }

        private void seedFrontier() {
            origin.travelCost = 0;
            origin.chargedMalus = 0;
            origin.estimate = heuristic(origin);
            origin.rank = origin.estimate;
            frontier.add(origin);
        }

        private boolean markArrivals(SearchNode node) {
            boolean arrived = false;
            if (singleTarget != null) {
                if (singleTarget.manhattanTo(node) > arrivalRadius) return false;
                singleTarget.markReached();
                return true;
            }
            for (TargetSelection target : targets) {
                if (target.manhattanTo(node) <= arrivalRadius) {
                    target.markReached();
                    arrived = true;
                }
            }
            return arrived;
        }

        private PathResult finishFound() {
            return priced(evaluator, PathStatus.FOUND,
                    selectedReached(), expanded, examined,
                    PathStop.REACHED);
        }

        private PathResult interruptedResult() {
            PathStatus status = control.cancelled() ? PathStatus.CANCELLED
                    : control.timedOut() ? PathStatus.TIMED_OUT : null;
            return status == null ? null : priced(evaluator, status,
                    selectedUnreached(), expanded, examined,
                    PathStop.FRONTIER_TRUNCATED);
        }

        private void consider(SearchNode current, SearchNode next) {
            if (next.settled) return;
            float segmentLength = evaluator.edgeCost(current, next);
            float travelled = current.routeLength + segmentLength;
            if (travelled >= distanceLimit) {
                if (!next.inOpenSet()) distanceRejected = true;
                return;
            }
            if (!evaluator.relaxable(next)) return;

            float penalty = evaluator.costMalus(next);
            float routeCost = current.travelCost + segmentLength + penalty;
            if (next.inOpenSet() && routeCost >= next.travelCost) return;

            next.previous = current;
            next.travelCost = routeCost;
            next.chargedMalus = penalty;
            next.routeLength = travelled;
            next.estimate = heuristic(next) * DESTINATION_PULL;
            float score = routeCost + next.estimate;
            if (next.inOpenSet()) frontier.lowerScore(next, score);
            else {
                next.rank = score;
                frontier.add(next);
            }
        }

        private float heuristic(SearchNode node) {
            if (singleTarget != null) return singleTarget.observe(node);
            return TargetSelection.observeAll(node, targets);
        }

        private SearchNode selectedReached() {
            return singleTarget == null
                    ? TargetSelection.selectReached(targets)
                    : singleTarget.bestNode();
        }

        private SearchNode selectedUnreached() {
            return singleTarget == null
                    ? TargetSelection.selectUnreached(targets)
                    : singleTarget.bestNode();
        }
    }

    /**
     * A frontier that survived the loop was cut short by the visit budget. One
     * that emptied is only proof of unreachability when the length bound never
     * withheld a cell, because that bound is measured from the start rather
     * than from the graph.
     */
    private static PathStop frontierStop(
            boolean emptied, boolean lengthWithheld) {
        if (!emptied) return PathStop.FRONTIER_TRUNCATED;
        return lengthWithheld
                ? PathStop.LENGTH_BOUNDED : PathStop.FRONTIER_EXHAUSTED;
    }

    private static PathResult priced(NodeEvaluator evaluator, PathStatus status,
                                     SearchNode end, int visited, int examined,
                                     PathStop stop) {
        return evaluator.result(status, end, visited, examined)
                .withNodeCosts(nodeCosts(end))
                .withStop(stop);
    }

    private static List<PathNodeCost> nodeCosts(SearchNode end) {
        List<PathNodeCost> costs = new ArrayList<>();
        for (SearchNode cursor = end; cursor != null; cursor = cursor.previous) {
            costs.add(new PathNodeCost(cursor.x, cursor.y, cursor.z,
                    cursor.type, cursor.chargedMalus));
        }
        Collections.reverse(costs);
        return costs;
    }
}
