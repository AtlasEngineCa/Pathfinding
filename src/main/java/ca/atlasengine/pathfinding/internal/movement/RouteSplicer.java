package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.result.PathNode;
import net.minestom.server.coordinate.Point;

import java.util.ArrayList;
import java.util.List;

/**
 * Plans a replacement route from where the entity will be when the search
 * lands rather than from where it stood when the search was submitted.
 *
 * <p>The splice node is the first node of the current route at least one
 * observed search latency of travel ahead. A plan starting there is joined to
 * the unconsumed prefix of the route the entity is already following, so the
 * seam is a node the entity literally stands on and the joined route stays
 * collision-legal end to end: every prefix edge was validated by the search
 * that produced it, and every suffix edge by the search that starts at the
 * seam. A plan that starts anywhere else is refused, and the caller replans
 * from the current position instead.</p>
 */
public final class RouteSplicer {
    /**
     * Fraction of the ground covered reaching the splice node that must also
     * be ground closed on the new target. One half admits the flanking curves
     * of an ordinary chase, which run up to sixty degrees off the bearing, and
     * rejects a target that has doubled back behind the entity, whose bearing
     * exceeds ninety degrees and closes nothing at all.
     */
    private static final double MINIMUM_PROGRESS_FRACTION = 0.5;
    /** Multiple of the predicted wait after which an armed splice expires. */
    private static final int DEADLINE_MULTIPLIER = 3;
    /** Floor of that deadline, so a fast system never churns splices. */
    private static final int MINIMUM_DEADLINE_TICKS = 20;
    /** Seam tolerance. The two routes must meet at one point, not near it. */
    private static final double SEAM_EPSILON = 1.0e-6;
    /** Recent submit-to-land waits kept to predict the next one. */
    private static final int LATENCY_SAMPLES = 8;

    /**
     * A joined route and how many consumed nodes were dropped from its front,
     * which the caller applies to every index it holds into the old route.
     */
    public record Join(List<PathNode> nodes, int offset) {
    }

    /**
     * Why armed splices did or did not become joined routes. {@code overrun}
     * counts the entities that passed the splice node before their plan
     * landed, {@code expired} those whose search outran its own predicted
     * wait, {@code superseded} the routes replaced while the search ran, and
     * {@code seamMismatch} the plans that did not start where they were asked
     * to. Each is a fallback to replanning from the current position, never a
     * discarded route.
     */
    public record Counts(long armed, long joined, long seamMismatch,
                         long overrun, long expired, long superseded,
                         long shortRoute, long divergent) {
    }

    private final int[] observedTicks = new int[LATENCY_SAMPLES];
    private int observedCursor;
    private int observedCount;

    private List<PathNode> armedRoute;
    private PathNode armedNode;
    private int armedIndex;
    private int armedTick;
    private int deadlineTicks;

    private long armed;
    private long joined;
    private long seamMismatch;
    private long overrun;
    private long expired;
    private long superseded;
    private long shortRoute;
    private long divergent;

    /**
     * Chooses the node the next search should start from and remembers it.
     * Returns {@code null} when the current route cannot carry a splice, in
     * which case the caller plans from the entity's own position.
     */
    public Point arm(List<PathNode> route, int nodeIndex, Point position,
                     Point target, double movementPerTick,
                     int latencyTicks, int tick) {
        disarm();
        if (route.isEmpty() || nodeIndex < 0 || nodeIndex >= route.size()
                || position == null || target == null || latencyTicks <= 0) {
            return null;
        }
        int index = firstNodeBeyond(
                route, nodeIndex, position, latencyTicks * movementPerTick);
        // The route is shorter than the entity will travel while the search
        // runs, so no node on it can carry the seam.
        if (index < 0) {
            shortRoute++;
            return null;
        }
        PathNode node = route.get(index);
        if (!closesOnTarget(position, node.asVec(), target)) {
            divergent++;
            return null;
        }
        armedRoute = route;
        armedIndex = index;
        armedNode = node;
        armedTick = tick;
        deadlineTicks = Math.max(MINIMUM_DEADLINE_TICKS,
                latencyTicks * DEADLINE_MULTIPLIER);
        armed++;
        return node.asVec();
    }

    /**
     * Whether search latency has grown enough for the seam to be worth moving
     * at all. While the entity drifts less than the start tolerance its
     * follower enforces, a plan computed from the position it occupies now
     * still lands close enough to be started, and its early waypoints still
     * lie ahead. Splicing below that threshold would retain a stretch of route
     * to solve a problem the follower does not yet have.
     */
    public static boolean worthSplicing(int latencyTicks,
                                        double movementPerTick,
                                        double startTolerance) {
        return latencyTicks * movementPerTick > startTolerance;
    }

    /** Records one observed submit-to-land wait, in whole ticks. */
    public void observeLatency(int ticks) {
        if (ticks < 0) return;
        observedTicks[observedCursor] = ticks;
        observedCursor = (observedCursor + 1) % LATENCY_SAMPLES;
        observedCount = Math.min(LATENCY_SAMPLES, observedCount + 1);
    }

    /**
     * Ticks the next search is expected to take, and so how far ahead the
     * seam belongs. Displacement is spent per tick rather than per
     * microsecond, so the waits this follower actually measured govern, and a
     * system-wide latency quantile only raises the estimate for a follower
     * that has not yet seen a search land.
     *
     * <p>The highest recent wait is taken rather than a mean, because a
     * follower's own samples under-report rather than over-report: its tick
     * clock advances only while it follows a route, so the waits it measures
     * before its first route lands read as no wait at all. A maximum ignores
     * those; a mean is dragged below the truth by them.</p>
     *
     * <p>This estimates the observed wait and no more. Placing the seam
     * further out buys more splices but commits the entity to longer stretches
     * of route aimed at older information, and measured across a latency sweep
     * that trade loses: the entity ends further from its target.</p>
     */
    public int expectedLatencyTicks(int reportedTicks) {
        int highest = Math.max(0, reportedTicks);
        for (int index = 0; index < observedCount; index++) {
            highest = Math.max(highest, observedTicks[index]);
        }
        return highest;
    }

    /** Whether a splice is waiting for its plan. */
    public boolean isArmed() {
        return armedRoute != null;
    }

    public void disarm() {
        armedRoute = null;
        armedNode = null;
        armedIndex = -1;
    }

    /**
     * Joins a landed plan to the route the entity is still following, or
     * returns {@code null} when it must be adopted from the current position
     * instead. Either way the armed splice is spent.
     */
    public Join join(List<PathNode> route, int nodeIndex,
                     List<PathNode> landed, int tick) {
        if (armedRoute == null) return null;
        List<PathNode> expected = armedRoute;
        PathNode seam = armedNode;
        int index = armedIndex;
        int deadline = deadlineTicks;
        int armedAt = armedTick;
        disarm();
        // Reference identity is exact here: a follower replaces its whole
        // node list whenever it adopts, so an unchanged list is the very
        // route the splice was measured against.
        if (route != expected || index >= route.size()
                || !route.get(index).equals(seam)) {
            superseded++;
            return null;
        }
        if (nodeIndex > index) {
            overrun++;
            return null;
        }
        if (tick - armedAt > deadline) {
            expired++;
            return null;
        }
        if (landed.size() < 2 || !meetsAt(landed.getFirst(), seam)) {
            seamMismatch++;
            return null;
        }
        // The route keeps its own seam node rather than taking the landed
        // plan's first one. They occupy the same cell at the same point, but
        // only the route's node carries the movement its predecessor was
        // planned to arrive by: a search starting on the far side of a gap
        // reports walking away from that cell, and adopting its node in place
        // of the jump that reaches the cell would strand the entity at the
        // edge of the gap.
        int offset = Math.max(0, nodeIndex - 1);
        List<PathNode> nodes =
                new ArrayList<>(index - offset + landed.size());
        nodes.addAll(route.subList(offset, index + 1));
        nodes.addAll(landed.subList(1, landed.size()));
        joined++;
        return new Join(List.copyOf(nodes), offset);
    }

    public Counts counts() {
        return new Counts(armed, joined, seamMismatch, overrun, expired,
                superseded, shortRoute, divergent);
    }

    /**
     * The first node at least {@code lookahead} blocks along the route from
     * the entity, measured through the nodes it will actually walk.
     */
    private static int firstNodeBeyond(List<PathNode> route, int nodeIndex,
                                       Point position, double lookahead) {
        double travelled = 0;
        Point cursor = position;
        for (int index = nodeIndex; index < route.size(); index++) {
            Point node = route.get(index).asVec();
            travelled += cursor.distance(node);
            cursor = node;
            if (travelled >= lookahead && carriesSeam(route.get(index))) {
                return index;
            }
        }
        return -1;
    }

    /**
     * Whether a route boundary may fall on this node. A jump or a climb is
     * executed over several ticks and leaves the follower holding state about
     * the node it is crossing, so a seam in the middle of one is the very
     * discontinuity splicing exists to avoid.
     */
    private static boolean carriesSeam(PathNode node) {
        return node.movement() != PathNode.Movement.JUMP
                && node.movement() != PathNode.Movement.CLIMB;
    }

    /**
     * Whether walking to the splice node is still worth doing now that the
     * target has moved. A route heading away from the new target closes none
     * of the distance to it, so splicing onto that route would graft a fresh
     * plan onto ground the entity should not be covering at all.
     */
    private static boolean closesOnTarget(
            Point position, Point splicePoint, Point target) {
        double covered = position.distance(splicePoint);
        double closed = position.distance(target) - splicePoint.distance(target);
        return closed >= MINIMUM_PROGRESS_FRACTION * covered;
    }

    private static boolean meetsAt(PathNode landed, PathNode seam) {
        return landed.graphX() == seam.graphX()
                && landed.graphY() == seam.graphY()
                && landed.graphZ() == seam.graphZ()
                && Math.abs(landed.x() - seam.x()) < SEAM_EPSILON
                && Math.abs(landed.y() - seam.y()) < SEAM_EPSILON
                && Math.abs(landed.z() - seam.z()) < SEAM_EPSILON;
    }
}
