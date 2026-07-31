package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.result.PathStatus;
import ca.atlasengine.pathfinding.search.SearchControl;
import ca.atlasengine.pathfinding.influence.InfluenceResult;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Shared volume-search evaluator. Terrain types are immutable per coordinate,
 * while node maluses intentionally persist and may increase each time the
 * evaluator requests the same coordinate.
 */
public abstract class SpatialNodeEvaluator extends NodeEvaluator {
    final NavigationRequest request;
    final SearchControl control;
    /** Search-scoped, so one search classifies under one set of answers. */
    final MobTraversalProfile mobProfile;
    final CoordinateNodeMap nodes = new CoordinateNodeMap();
    private final Map<CellKey, InfluenceResult> influenceResults =
            new HashMap<>();

    SpatialNodeEvaluator(NavigationRequest request, SearchControl control) {
        this.request = request;
        this.control = control;
        this.mobProfile = request.profile().mobProfile()
                .withMemoizedClassification();
    }

    abstract SearchNode accepted(Point point);

    abstract PathNode.Movement movement(SearchNode node);

    final InfluenceResult combinedInfluence(Point graphPoint) {
        if (request.influences().isEmpty()) return InfluenceResult.NONE;
        return influenceResults.computeIfAbsent(CellKey.of(graphPoint), ignored -> {
            double cost = 0;
            Point movement = movementPoint(request, graphPoint);
            for (NavigationInfluence influence : request.influences()) {
                InfluenceResult result = influence.evaluate(
                        request.blocks(), movement,
                        request.boundingBox(), control);
                if (result.blocked()) return result;
                cost += result.costDelta();
            }
            return cost == 0 ? InfluenceResult.NONE
                    : InfluenceResult.penalty(cost, "combined_influences");
        });
    }

    float malusAt(Point point) {
        SearchNode node = accepted(point);
        return node == null ? -1 : (float) node.malus;
    }

    final SearchNode node(Point point) {
        SearchNode node = nodes.getOrCreate(
                point.blockX(), point.blockY(), point.blockZ());
        if (node.malus == Double.NEGATIVE_INFINITY) {
            node.malus = 0;
        }
        return node;
    }

    static Point point(SearchNode node) {
        return new Vec(node.x, node.y, node.z);
    }

    private double combinedMalus(SearchNode neighbor) {
        InfluenceResult influenceResult = combinedInfluence(point(neighbor));
        return influenceResult.blocked()
                ? -1 : neighbor.malus + influenceResult.costDelta();
    }

    @Override
    final boolean relaxable(SearchNode neighbor) {
        return combinedMalus(neighbor) >= 0;
    }

    @Override
    final float costMalus(SearchNode neighbor) {
        return (float) Math.max(0, combinedMalus(neighbor));
    }

    @Override
    final float edgeCost(SearchNode from, SearchNode to) {
        int dx = to.x - from.x;
        int dy = to.y - from.y;
        int dz = to.z - from.z;
        return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    @Override
    final boolean exceedsPathLength(
            SearchNode start, SearchNode current, double maxPathLength) {
        return distance(current, start) >= maxPathLength;
    }

    @Override
    final PathResult result(PathStatus status, SearchNode end,
                            int visited, int examined) {
        List<PathNode> nodes = new ArrayList<>();
        for (SearchNode node = end; node != null; node = node.previous) {
            Point waypoint = movementPoint(request, point(node));
            nodes.add(new PathNode(
                    waypoint.x(), waypoint.y(), waypoint.z(),
                    movement(node), node.x, node.y, node.z));
        }
        Collections.reverse(nodes);
        return new PathResult(status, nodes, visited, examined);
    }

    static Point movementPoint(NavigationRequest request, Point graphPoint) {
        int width = Math.max(1,
                (int) Math.floor(request.boundingBox().width() + 1));
        int depth = Math.max(1,
                (int) Math.floor(request.boundingBox().depth() + 1));
        return new Vec(graphPoint.blockX() + width * 0.5,
                graphPoint.blockY(), graphPoint.blockZ() + depth * 0.5);
    }

    private static float distance(SearchNode a, SearchNode b) {
        float x = b.x - a.x;
        float y = b.y - a.y;
        float z = b.z - a.z;
        return (float) Math.sqrt(x * x + y * y + z * z);
    }
}
