package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.result.PathStop;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;

import java.util.List;

/**
 * Replanning policy for routes that end short of the target: how far a search
 * may be widened, and when another bounded segment is worth requesting.
 */
public final class PartialRouteReplanner {
    /** Upper bound on adaptive search-range growth. */
    private static final double MAX_SEARCH_RANGE_MULTIPLIER = 4;
    /** Rise above the route endpoint that justifies a wider search. */
    private static final double VERTICAL_DETOUR_RISE = 4;

    public enum Continuation { STOP, WIDEN, REPLAN }

    private double partialSegmentStartDistance;
    private Point lastPartialEndpoint;
    private double searchRangeMultiplier = 1;

    public double searchRangeMultiplier() {
        return searchRangeMultiplier;
    }

    public void reset(double distanceToTarget) {
        partialSegmentStartDistance = distanceToTarget;
        lastPartialEndpoint = null;
        searchRangeMultiplier = 1;
    }

    public void beginSegment(double distanceToTarget) {
        partialSegmentStartDistance = distanceToTarget;
    }

    /**
     * Doubles the search budget.
     */
    public void widen() {
        searchRangeMultiplier = Math.min(MAX_SEARCH_RANGE_MULTIPLIER,
                searchRangeMultiplier * 2);
    }

    /**
     * Chains bounded partial searches for long routes. A nearby
     * unreachable target remains terminal, and an identical partial endpoint
     * is never submitted twice, preventing autonomous retry loops.
     *
     * <p>For a route whose search reason is not recorded, such as one replayed
     * from a cached plan.</p>
     */
    public Continuation continueLongPartialRoute(
            Point position, Point target, List<PathNode> nodes) {
        return continueLongPartialRoute(
                position, target, nodes, PathStop.NOT_SEARCHED);
    }

    /**
     * Chains bounded partial searches for long routes. A nearby
     * unreachable target remains terminal, and an identical partial endpoint
     * is never submitted twice, preventing autonomous retry loops.
     *
     * <p>The terminal radius reads a straight line, so it only answers a
     * search for which a straight line is the right measure. A search whose
     * frontier was cut short by its budget after walking further than the
     * target is away has shown the opposite, and chains inside the radius. The
     * two remaining guards still bound that: a segment must close two blocks
     * of distance, and an endpoint is never submitted twice, so a target that
     * cannot be reached costs at most one further segment rather than a
     * loop.</p>
     */
    public Continuation continueLongPartialRoute(
            Point position, Point target, List<PathNode> nodes, PathStop stop) {
        if (target == null || nodes.isEmpty()) return Continuation.STOP;
        double remaining = position.distance(target);
        if ((remaining <= 8 && !budgetBound(stop, nodes, remaining))
                || partialSegmentStartDistance - remaining < 2) {
            if (target.y() - position.y() > VERTICAL_DETOUR_RISE
                    && searchRangeMultiplier < MAX_SEARCH_RANGE_MULTIPLIER) {
                return Continuation.WIDEN;
            }
            return Continuation.STOP;
        }
        PathNode endpoint = nodes.getLast();
        Point graphEndpoint = new Vec(
                endpoint.graphX(), endpoint.graphY(), endpoint.graphZ());
        if (lastPartialEndpoint != null
                && lastPartialEndpoint.distance(graphEndpoint) < 1.0e-9) {
            return Continuation.STOP;
        }
        lastPartialEndpoint = graphEndpoint;
        return Continuation.REPLAN;
    }

    /**
     * Whether another bounded segment would search graph this one could not
     * reach, which is the only thing that earns an exception to the radius.
     *
     * <p>The frontier must have been cut short rather than emptied.
     * {@link PathStop#FRONTIER_EXHAUSTED} proves the target unreachable from
     * here. {@link PathStop#LENGTH_BOUNDED} is excluded too: its frontier did
     * empty, so all it says is that a bound already reaching well past a
     * target this close withheld cells somewhere, which is the geometry the
     * radius exists to call terminal.</p>
     *
     * <p>The route just walked must also be longer than the straight line
     * still to go. A cut-short frontier only means the search stopped
     * somewhere; a segment shorter than the distance remaining stopped
     * because it had nowhere better to go, not because eight blocks of
     * straight line understated the walk.</p>
     */
    private static boolean budgetBound(
            PathStop stop, List<PathNode> nodes, double remaining) {
        return stop == PathStop.FRONTIER_TRUNCATED
                && walkedLength(nodes) > remaining;
    }

    private static double walkedLength(List<PathNode> nodes) {
        double walked = 0;
        for (int index = 1; index < nodes.size(); index++) {
            walked += nodes.get(index - 1).asVec()
                    .distance(nodes.get(index).asVec());
        }
        return walked;
    }

    public boolean shouldExpandVerticalDetour(PathResult path, Point target) {
        if (target == null || path.nodes().isEmpty()
                || searchRangeMultiplier >= MAX_SEARCH_RANGE_MULTIPLIER) return false;
        PathNode endpoint = path.nodes().getLast();
        double horizontal = Math.hypot(
                endpoint.x() - target.x(), endpoint.z() - target.z());
        return target.y() - endpoint.y() > VERTICAL_DETOUR_RISE && horizontal < 4;
    }
}
