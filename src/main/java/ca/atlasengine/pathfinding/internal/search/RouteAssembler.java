package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.result.PathStatus;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Turns the settled parent chain into the route a follower drives.
 *
 * <p>Two translations happen here, and they are separate on purpose. Graph
 * coordinates are integer cell anchors; movement waypoints are real positions
 * an entity can be steered toward, which means projecting onto the collision
 * floor and to the centre of the footprint. Only after that projection is the
 * <em>movement</em> of each hop knowable, because whether a hop is a step up
 * or a fall depends on the projected heights, not on the cell indices.</p>
 *
 * <p>Movement describes <em>entering</em> a node, so it can only be assigned
 * once the chain runs start-to-end. The chain is walked backwards from the
 * goal, so the pass below reverses first and tags second.</p>
 */
final class RouteAssembler {
    /** Height slack below which two waypoints count as level. */
    private static final double LEVEL_EPSILON = 1.0e-7;

    private final GroundNodeEvaluator evaluator;
    private final ClimbSupport climb;
    private final BoundingBox box;

    RouteAssembler(GroundNodeEvaluator evaluator, ClimbSupport climb) {
        this.evaluator = evaluator;
        this.climb = climb;
        this.box = evaluator.box;
    }

    PathResult assemble(PathStatus status, SearchNode end,
                        int visited, int examined) {
        List<PathNode> projected = project(end);
        Collections.reverse(projected);
        return new PathResult(status, tag(projected), visited, examined);
    }

    /** Graph cells to movement waypoints, goal-first. */
    private List<PathNode> project(SearchNode end) {
        List<PathNode> waypoints = new ArrayList<>();
        double halfWidth = GroundNodeEvaluator.footprintWidth(box) * 0.5;
        double halfDepth = GroundNodeEvaluator.footprintDepth(box) * 0.5;
        for (SearchNode cursor = end; cursor != null; cursor = cursor.previous) {
            waypoints.add(new PathNode(
                    climb.waypointX(cursor.x, cursor.y, cursor.z, halfWidth),
                    evaluator.waypointY(cursor.x, cursor.y, cursor.z),
                    climb.waypointZ(cursor.x, cursor.y, cursor.z, halfDepth),
                    PathNode.Movement.WALK,
                    cursor.x, cursor.y, cursor.z));
        }
        return waypoints;
    }

    /** Assigns each waypoint the movement that arrives at it. */
    private List<PathNode> tag(List<PathNode> waypoints) {
        List<PathNode> tagged = new ArrayList<>(waypoints.size());
        for (int i = 0; i < waypoints.size(); i++) {
            PathNode previous = i == 0 ? null : waypoints.get(i - 1);
            PathNode node = waypoints.get(i);
            // The first node is arrived at by standing still.
            PathNode.Movement fallback = previous == null
                    ? PathNode.Movement.WALK : geometricMovement(previous, node);
            tagged.add(new PathNode(node.x(), node.y(), node.z(),
                    override(previous, node, fallback),
                    node.graphX(), node.graphY(), node.graphZ()));
        }
        return tagged;
    }

    /**
     * Movements the terrain dictates regardless of geometry. A climb is a held
     * ascent rather than a step, and an amphibious mob in water swims whatever
     * the height delta looks like.
     */
    private PathNode.Movement override(
            PathNode previous, PathNode node, PathNode.Movement fallback) {
        if (climb.isClimbMove(previous, node)) return PathNode.Movement.CLIMB;
        boolean swimming = evaluator.isAmphibious()
                && evaluator.type(node.graphX(), node.graphY(), node.graphZ())
                        == TerrainType.WATER;
        return swimming ? PathNode.Movement.SWIM : fallback;
    }

    /** Movement implied purely by where the two waypoints sit. */
    private static PathNode.Movement geometricMovement(
            PathNode previous, PathNode node) {
        // More than one cell of horizontal travel can only be a jump arc;
        // ordinary edges are between adjacent cells.
        if (Math.abs(node.graphX() - previous.graphX()) > 1
                || Math.abs(node.graphZ() - previous.graphZ()) > 1) {
            return PathNode.Movement.JUMP;
        }
        if (node.y() > previous.y() + LEVEL_EPSILON) {
            return PathNode.Movement.STEP_UP;
        }
        if (node.y() < previous.y() - LEVEL_EPSILON) {
            return PathNode.Movement.FALL;
        }
        return PathNode.Movement.WALK;
    }
}
