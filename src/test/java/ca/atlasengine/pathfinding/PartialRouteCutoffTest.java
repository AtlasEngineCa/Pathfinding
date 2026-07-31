package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.internal.movement.PartialRouteReplanner;
import ca.atlasengine.pathfinding.internal.movement.PartialRouteReplanner.Continuation;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Pins the decision table behind chained partial routes, so the eight-block
 * radius that makes a nearby target terminal cannot move by accident.
 *
 * <p>The radius is deliberate policy rather than a baseline constant: Minecraft
 * has no partial-route chaining at all, and its own eight in
 * {@code PathNavigation.createPath} is the search region's padding. Widening
 * the radius here would let a nearby unreachable target be submitted again,
 * which is exactly the autonomous retry loop the chaining rules exist to
 * prevent.</p>
 */
class PartialRouteCutoffTest {
    private static final Point ORIGIN = new Vec(0, 1, 0);

    @Test
    void chainsOnlyWhatIsFurtherAwayThanTheTerminalRadius() {
        PartialRouteReplanner replanner = new PartialRouteReplanner();
        replanner.reset(40);

        replanner.beginSegment(11);
        assertEquals(Continuation.REPLAN, replanner.continueLongPartialRoute(
                        ORIGIN, target(8.0001), route(3)),
                "a target past the radius still deserves another segment");

        replanner.beginSegment(11);
        assertEquals(Continuation.STOP, replanner.continueLongPartialRoute(
                        ORIGIN, target(8), route(4)),
                "a target inside the radius is terminal however productive "
                        + "the segment that ended there was");
    }

    @Test
    void refusesToChainASegmentThatClosedLessThanTwoBlocks() {
        PartialRouteReplanner replanner = new PartialRouteReplanner();
        replanner.reset(40);

        replanner.beginSegment(21.999);
        assertEquals(Continuation.STOP, replanner.continueLongPartialRoute(
                ORIGIN, target(20), route(3)));

        replanner.beginSegment(22);
        assertEquals(Continuation.REPLAN, replanner.continueLongPartialRoute(
                ORIGIN, target(20), route(4)));
    }

    @Test
    void neverSubmitsAnIdenticalPartialEndpointTwice() {
        PartialRouteReplanner replanner = new PartialRouteReplanner();
        replanner.reset(40);

        replanner.beginSegment(22);
        assertEquals(Continuation.REPLAN, replanner.continueLongPartialRoute(
                ORIGIN, target(20), route(5)));

        replanner.beginSegment(22);
        assertEquals(Continuation.STOP, replanner.continueLongPartialRoute(
                ORIGIN, target(20), route(5)));

        // A new navigation forgets the endpoint the previous one exhausted.
        replanner.reset(40);
        replanner.beginSegment(22);
        assertEquals(Continuation.REPLAN, replanner.continueLongPartialRoute(
                ORIGIN, target(20), route(5)));
    }

    @Test
    void aRoutelessOrTargetlessFollowerIsTerminal() {
        PartialRouteReplanner replanner = new PartialRouteReplanner();
        replanner.reset(40);
        replanner.beginSegment(22);

        assertEquals(Continuation.STOP, replanner.continueLongPartialRoute(
                ORIGIN, null, route(5)));
        assertEquals(Continuation.STOP, replanner.continueLongPartialRoute(
                ORIGIN, target(20), List.of()));
    }

    @Test
    void aTargetHighAboveTheRadiusWidensUntilTheBudgetCeiling() {
        PartialRouteReplanner replanner = new PartialRouteReplanner();
        replanner.reset(40);
        Point overhead = new Vec(0, 6, 0);

        replanner.beginSegment(9);
        assertEquals(Continuation.WIDEN, replanner.continueLongPartialRoute(
                        ORIGIN, overhead, route(3)),
                "a rise the search could not cover is worth a wider budget");

        assertEquals(1, replanner.searchRangeMultiplier());
        replanner.widen();
        assertEquals(2, replanner.searchRangeMultiplier());
        replanner.widen();
        assertEquals(4, replanner.searchRangeMultiplier());
        replanner.widen();
        assertEquals(4, replanner.searchRangeMultiplier(),
                "the search budget must not grow without bound");

        replanner.beginSegment(9);
        assertEquals(Continuation.STOP, replanner.continueLongPartialRoute(
                        ORIGIN, overhead, route(4)),
                "an exhausted budget makes the rise terminal too");
    }

    private static Point target(double distance) {
        return new Vec(distance, 1, 0);
    }

    private static List<PathNode> route(int endpointX) {
        return List.of(
                new PathNode(0.5, 1, 0.5, PathNode.Movement.WALK, 0, 1, 0),
                new PathNode(endpointX + 0.5, 1, 0.5,
                        PathNode.Movement.WALK, endpointX, 1, 0));
    }
}
