package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.result.PathNode;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The splice decision in isolation: where the seam goes, when a landed plan
 * may be joined to the route still being followed, and every way that must
 * fall back to planning from the entity's own position instead.
 */
class RouteSplicerTest {
    private static final double MOVEMENT_PER_TICK = 0.2;
    private static final double START_TOLERANCE = 1.5;

    private final RouteSplicer splicer = new RouteSplicer();

    /**
     * A search that lands before the entity has drifted past the tolerance its
     * follower enforces needs no splice: the plan it produces still starts
     * close enough to be begun.
     */
    @Test
    void latencyInsideTheStartToleranceIsNotWorthSplicing() {
        assertFalse(RouteSplicer.worthSplicing(
                7, MOVEMENT_PER_TICK, START_TOLERANCE));
        assertTrue(RouteSplicer.worthSplicing(
                8, MOVEMENT_PER_TICK, START_TOLERANCE));
    }

    /** Failure mode: the route is shorter than the entity will travel. */
    @Test
    void aRouteShorterThanTheLookaheadCarriesNoSeam() {
        List<PathNode> route = straightRoute(4);

        assertNull(arm(route, 1, at(0.5), at(40.5), 40),
                "a four-node route cannot carry a seam eight blocks along");
        assertEquals(1, splicer.counts().shortRoute());
        assertEquals(0, splicer.counts().armed());
    }

    /** Failure mode: the target doubled back behind the entity. */
    @Test
    void aTargetBehindTheEntityRefusesTheSplice() {
        List<PathNode> route = straightRoute(20);

        assertNull(arm(route, 1, at(0.5), at(-20.5), 50),
                "a route running away from the target must not carry a seam");
        assertEquals(1, splicer.counts().divergent());
    }

    /**
     * The direction test admits the flanking curve of an ordinary chase and
     * refuses only a route that closes too little of the distance to be worth
     * the ground it covers.
     */
    @Test
    void aFlankingRouteStillCarriesASeam() {
        List<PathNode> route = straightRoute(20);

        assertNotNull(arm(route, 1, at(0.5), new Vec(14.5, 40, 6.0), 50),
                "a target ahead and to one side must still splice");
    }

    /** Failure mode: the entity passed the seam before its plan landed. */
    @Test
    void anEntityPastTheSeamFallsBackInsteadOfSplicing() {
        List<PathNode> route = straightRoute(20);
        Point seam = arm(route, 1, at(0.5), at(20.5), 50);
        assertNotNull(seam);

        assertNull(splicer.join(route, 12, planFrom(seam, 6), 5),
                "an entity past the seam must not splice");
        assertEquals(1, splicer.counts().overrun());
    }

    /**
     * Failure mode: the entity never reaches the seam. A position check alone
     * would wait forever, so the splice also expires.
     */
    @Test
    void aSpliceOutlivingItsDeadlineIsAbandoned() {
        List<PathNode> route = straightRoute(20);
        Point seam = arm(route, 1, at(0.5), at(20.5), 50);
        assertNotNull(seam);

        assertNull(splicer.join(route, 1, planFrom(seam, 6), 500),
                "a splice whose search never landed must expire");
        assertEquals(1, splicer.counts().expired());
    }

    /** Failure mode: the plan did not start where it was asked to. */
    @Test
    void aPlanThatDoesNotStartAtTheSeamIsRefused() {
        List<PathNode> route = straightRoute(20);
        Point seam = arm(route, 1, at(0.5), at(20.5), 50);
        assertNotNull(seam);

        assertNull(splicer.join(route, 1,
                        planFrom(seam.add(1, 0, 0), 6), 5),
                "a plan starting beside the seam is not a splice");
        assertEquals(1, splicer.counts().seamMismatch());
        assertEquals(0, splicer.counts().joined());
    }

    /** Failure mode: the route the splice was measured against was replaced. */
    @Test
    void aReplacedRouteRefusesTheSplice() {
        List<PathNode> route = straightRoute(20);
        Point seam = arm(route, 1, at(0.5), at(20.5), 50);
        assertNotNull(seam);

        assertNull(splicer.join(straightRoute(20), 1, planFrom(seam, 6), 5),
                "an equal but different route is not the armed one");
        assertEquals(1, splicer.counts().superseded());
    }

    @Test
    void aJoinedRouteIsContinuousAcrossTheSeam() {
        List<PathNode> route = straightRoute(24);
        Point seam = arm(route, 5, at(4.5), at(24.5), 50);
        assertNotNull(seam);
        assertEquals(14.5, seam.x(), 1.0e-9);

        RouteSplicer.Join join = splicer.join(route, 5, planFrom(seam, 6), 5);
        assertNotNull(join);
        assertEquals(1, splicer.counts().joined());
        List<PathNode> joined = join.nodes();
        for (int index = 1; index < joined.size(); index++) {
            PathNode previous = joined.get(index - 1);
            PathNode current = joined.get(index);
            assertEquals(1.0, previous.asVec().distance(current.asVec()), 1.0e-9,
                    () -> "discontinuous seam: " + previous + " -> " + current);
        }
        // Consumed nodes are dropped, so a chase of any length keeps a
        // bounded route, and the follower's own index still addresses the
        // waypoint it was walking toward.
        assertEquals(4, join.offset(), "the consumed prefix was not trimmed");
        assertEquals(route.get(5), joined.get(5 - join.offset()));
        assertEquals(route.get(4), joined.getFirst(),
                "the node behind the entity was not preserved");
    }

    /**
     * A seam takes the movement of the node the route already planned to
     * arrive by. A search starting on the far side of a gap reports walking
     * away from that cell, and adopting its node in place of the jump that
     * reaches the cell would strand the entity at the gap's edge.
     */
    @Test
    void aJoinedRouteKeepsTheSeamNodesOwnMovement() {
        List<PathNode> route = new ArrayList<>(straightRoute(20));
        route.set(10, new PathNode(10.5, 40, 0.5, PathNode.Movement.FALL,
                10, 40, 0));
        List<PathNode> fixed = List.copyOf(route);
        Point seam = arm(fixed, 1, at(0.5), at(20.5), 50);
        assertNotNull(seam);
        assertEquals(10.5, seam.x(), 1.0e-9);

        RouteSplicer.Join join = splicer.join(fixed, 1, planFrom(seam, 6), 5);
        assertNotNull(join);
        PathNode joinedSeam = join.nodes().get(10 - join.offset());
        assertEquals(PathNode.Movement.FALL, joinedSeam.movement(),
                "the landed plan's walking start replaced the route's own "
                        + "arrival movement");
    }

    /** A jump or a climb is crossed over several ticks and cannot be a seam. */
    @Test
    void aSeamIsNeverPlacedOnAJumpOrAClimb() {
        for (PathNode.Movement movement : List.of(
                PathNode.Movement.JUMP, PathNode.Movement.CLIMB)) {
            List<PathNode> route = new ArrayList<>(straightRoute(13));
            for (int index = 10; index < 13; index++) {
                route.set(index, new PathNode(index + 0.5, 40, 0.5, movement,
                        index, 40, 0));
            }
            RouteSplicer fresh = new RouteSplicer();

            assertNull(fresh.arm(List.copyOf(route), 1, at(0.5), at(20.5),
                            MOVEMENT_PER_TICK, 50, 0),
                    () -> "a " + movement + " node carried a seam");
        }
    }

    /**
     * A follower's tick clock advances only while it follows a route, so the
     * waits it measures before its first route lands read as no wait at all.
     * A maximum ignores those; a mean would be dragged below the truth.
     */
    @Test
    void theExpectedWaitIgnoresTheZeroesReportedBeforeFollowingBegins() {
        splicer.observeLatency(0);
        splicer.observeLatency(0);
        splicer.observeLatency(20);

        assertEquals(20, splicer.expectedLatencyTicks(0));
        assertEquals(31, splicer.expectedLatencyTicks(31),
                "a system-wide quantile must still raise a low estimate");
    }

    @Test
    void anArmedSpliceIsSpentWhetherOrNotItJoins() {
        List<PathNode> route = straightRoute(20);
        Point seam = arm(route, 1, at(0.5), at(20.5), 50);
        assertNotNull(seam);
        assertTrue(splicer.isArmed());
        assertEquals(10.5, seam.x(), 1.0e-9);

        splicer.join(route, 1, planFrom(seam, 6), 5);
        assertFalse(splicer.isArmed(), "a spent splice stayed armed");
        assertNull(splicer.join(route, 1, planFrom(seam, 6), 5),
                "a spent splice joined a second plan");
    }

    private Point arm(List<PathNode> route, int nodeIndex, Point position,
                      Point target, int latencyTicks) {
        return splicer.arm(route, nodeIndex, position, target,
                MOVEMENT_PER_TICK, latencyTicks, 0);
    }

    private static Point at(double x) {
        return new Vec(x, 40, 0.5);
    }

    /** A straight eastward walking route on one-block spacing. */
    private static List<PathNode> straightRoute(int length) {
        List<PathNode> nodes = new ArrayList<>(length);
        for (int index = 0; index < length; index++) {
            nodes.add(new PathNode(index + 0.5, 40, 0.5,
                    PathNode.Movement.WALK, index, 40, 0));
        }
        return List.copyOf(nodes);
    }

    /** A landed plan continuing eastward from its own start node. */
    private static List<PathNode> planFrom(Point start, int length) {
        List<PathNode> nodes = new ArrayList<>(length);
        for (int index = 0; index < length; index++) {
            double x = start.x() + index;
            nodes.add(new PathNode(x, start.y(), start.z(),
                    PathNode.Movement.WALK, (int) Math.floor(x),
                    (int) Math.floor(start.y()),
                    (int) Math.floor(start.z())));
        }
        return List.copyOf(nodes);
    }
}
