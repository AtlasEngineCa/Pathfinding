package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.result.PathStatus;
import ca.atlasengine.pathfinding.search.SearchControl;
import net.minestom.server.coordinate.Vec;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PathSearchWorkLimitTest {
    @Test
    void absoluteVisitLimitCapsOtherwiseUnboundedBudgets() {
        EndlessEvaluator evaluator = new EndlessEvaluator();

        PathResult result = PathSearch.find(evaluator,
                List.of(new Vec(1_000_000, 0, 0)), Double.MAX_VALUE, 0,
                Integer.MAX_VALUE, SearchControl.NONE);

        assertEquals(PathStatus.PARTIAL, result.status());
        assertTrue(result.visitedNodes() <= PathSearch.MAX_VISITED_NODES);
        assertTrue(result.examinedNeighbors() <= PathSearch.MAX_VISITED_NODES);
    }

    private static final class EndlessEvaluator extends NodeEvaluator {
        private final SearchNode start = new SearchNode(0, 0, 0);

        private EndlessEvaluator() {
            start.malus = 0;
        }

        @Override
        SearchNode getStart() {
            return start;
        }

        @Override
        List<SearchNode> getNeighbors(SearchNode current) {
            SearchNode next = new SearchNode(current.x + 1, 0, 0);
            next.malus = 0;
            return List.of(next);
        }

        @Override
        int lastExaminedNeighbors() {
            return 1;
        }

        @Override
        float edgeCost(SearchNode from, SearchNode to) {
            return 1;
        }

        @Override
        boolean exceedsPathLength(
                SearchNode start, SearchNode current, double maxPathLength) {
            return false;
        }

        @Override
        PathResult result(PathStatus status, SearchNode end,
                          int visited, int examined) {
            return new PathResult(status, List.of(), visited, examined);
        }
    }
}
