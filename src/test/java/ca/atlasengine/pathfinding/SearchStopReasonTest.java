package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.result.PathStop;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.internal.movement.PartialRouteReplanner;
import ca.atlasengine.pathfinding.internal.movement.PartialRouteReplanner.Continuation;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

/**
 * Separates the three ways a partial search ends, and pins what each one is
 * allowed to do to the terminal radius.
 *
 * <p>{@code PathStatus.PARTIAL} says only that no destination was reached. An
 * emptied frontier proves the target unreachable from the start it was given;
 * a frontier cut short by the visit budget proves nothing of the sort; and a
 * frontier emptied only because the length bound withheld cells is a third
 * case that must not be mistaken for either.</p>
 */
class SearchStopReasonTest {
    private static final BoundingBox BOX = new BoundingBox(0.6, 1.95, 0.6);
    private static final Point ORIGIN = new Vec(0, 1, 0);

    @Test
    void aReachedRouteNamesItsOwnStop() {
        TestWorld world = new TestWorld().floor(-2, 12, -2, 2, 0, Block.STONE);

        PathResult result = search(world, new Vec(0.5, 1, 0.5),
                new Vec(8.5, 1, 0.5), 64, 1);

        assertEquals(PathStatus.FOUND, result.status());
        assertEquals(PathStop.REACHED, result.stop());
    }

    @Test
    void anEmptiedFrontierWithNothingWithheldIsExhausted() {
        // A sealed pen far smaller than either budget: the search expands
        // every cell it can reach and never touches a bound.
        TestWorld world = new TestWorld().floor(-3, 3, -3, 3, 0, Block.STONE);
        wall(world, 4);

        PathResult result = search(world, new Vec(0.5, 1, 0.5),
                new Vec(20.5, 1, 0.5), 64, 1);

        assertEquals(PathStatus.PARTIAL, result.status());
        assertEquals(PathStop.FRONTIER_EXHAUSTED, result.stop(),
                "a pen inside every budget can only end by running out of "
                        + "graph");
    }

    @Test
    void aVisitBudgetThatBitesFirstLeavesTheFrontierTruncated() {
        // Open ground the search cannot finish: the visit budget stops the
        // loop with cells still queued.
        TestWorld world =
                new TestWorld().floor(-40, 40, -40, 40, 0, Block.STONE);
        wall(world, 41);

        PathResult result = search(world, new Vec(0.5, 1, 0.5),
                new Vec(39.5, 1, 39.5), 512, 0.002);

        assertEquals(PathStatus.PARTIAL, result.status());
        assertEquals(PathStop.FRONTIER_TRUNCATED, result.stop(),
                "cells left queued are not an exhausted frontier");
    }

    /**
     * The length-pruning case. The frontier empties, but only because the
     * bound refused cells, so calling it exhausted would claim an unreachable
     * target the geometry never proved.
     */
    @Test
    void aFrontierEmptiedOnlyByTheLengthBoundIsNotExhausted() {
        TestWorld world =
                new TestWorld().floor(-2, 200, -2, 2, 0, Block.STONE);
        wall(world, 201);

        PathResult result = search(world, new Vec(0.5, 1, 0.5),
                new Vec(180.5, 1, 0.5), 12, 8);

        assertEquals(PathStatus.PARTIAL, result.status());
        assertEquals(PathStop.LENGTH_BOUNDED, result.stop());
        assertNotEquals(PathStop.FRONTIER_EXHAUSTED, result.stop(),
                "a cell the length bound refused is not a cell that "
                        + "does not exist");
    }

    /**
     * The other length-pruning site. A climber charged only horizontal edge
     * cost walks no distance at all going up, so the bound can only stop it
     * where an expansion is refused rather than where a neighbour is. That
     * frontier empties with cells withheld just the same.
     */
    @Test
    void aClimbRefusedByTheBoundIsWithheldToo() {
        TestWorld world = shaft(60);

        PathResult result = climb(world, new Vec(0.5, 1, 0.5),
                new Vec(20.5, 1, 20.5), 8);

        assertEquals(PathStatus.PARTIAL, result.status());
        assertEquals(PathStop.LENGTH_BOUNDED, result.stop(),
                "a climb the bound refused to expand is a withheld cell");
    }

    @Test
    void aTruncatedFrontierChainsInsideTheTerminalRadius() {
        PartialRouteReplanner replanner = radiusCase();

        assertEquals(Continuation.REPLAN, replanner.continueLongPartialRoute(
                        ORIGIN, target(8), longSegment(),
                        PathStop.FRONTIER_TRUNCATED),
                "a budget that ran out inside the radius is not a target "
                        + "that cannot be reached");
    }

    @Test
    void anExhaustedFrontierStaysTerminalInsideTheRadius() {
        PartialRouteReplanner replanner = radiusCase();

        assertEquals(Continuation.STOP, replanner.continueLongPartialRoute(
                        ORIGIN, target(8), longSegment(),
                        PathStop.FRONTIER_EXHAUSTED),
                "an emptied frontier proves the target unreachable from "
                        + "here, however long the segment that ended there");
    }

    @Test
    void aLengthBoundedFrontierStaysTerminalInsideTheRadius() {
        PartialRouteReplanner replanner = radiusCase();

        assertEquals(Continuation.STOP, replanner.continueLongPartialRoute(
                        ORIGIN, target(8), longSegment(),
                        PathStop.LENGTH_BOUNDED),
                "a bound already reaching past a target this close must not "
                        + "buy another segment");
    }

    /**
     * The island shape: a short hop that ended inside the radius with cells
     * still queued somewhere irrelevant. Straight-line distance is the honest
     * measure of that segment, so the radius still answers it.
     */
    @Test
    void aSegmentShorterThanTheDistanceLeftStaysTerminal() {
        PartialRouteReplanner replanner = new PartialRouteReplanner();
        replanner.reset(40);
        replanner.beginSegment(6);

        assertEquals(Continuation.STOP, replanner.continueLongPartialRoute(
                        new Vec(2.5, 1, 0), target(6.5), route(0, 2),
                        PathStop.FRONTIER_TRUNCATED),
                "walking two blocks to stop four short is not a budget the "
                        + "radius should yield to");
    }

    @Test
    void theReasonFreeOverloadKeepsTheShippedRadius() {
        PartialRouteReplanner replanner = radiusCase();

        assertEquals(Continuation.STOP, replanner.continueLongPartialRoute(
                        ORIGIN, target(8), longSegment()),
                "a route with no recorded reason falls back to the radius");
    }

    /**
     * The retry-loop guard. A follower that cannot close ground gets one
     * chained segment out of the lifted radius and no more, and the endpoint
     * it stopped on is never submitted twice.
     */
    @Test
    void aChainedSegmentThatClosesNothingIsTerminal() {
        PartialRouteReplanner replanner = new PartialRouteReplanner();
        replanner.reset(40);
        replanner.beginSegment(30);
        Point stopped = new Vec(0, 1, 0);

        assertEquals(Continuation.REPLAN, replanner.continueLongPartialRoute(
                stopped, target(8), longSegment(),
                PathStop.FRONTIER_TRUNCATED));

        replanner.beginSegment(8);
        assertEquals(Continuation.STOP, replanner.continueLongPartialRoute(
                        stopped, target(8), longSegment(),
                        PathStop.FRONTIER_TRUNCATED),
                "a segment that closed no ground ends the chain whatever "
                        + "the frontier did");
    }

    @Test
    void aChainedSegmentNeverResubmitsAnEndpoint() {
        PartialRouteReplanner replanner = new PartialRouteReplanner();
        replanner.reset(40);
        replanner.beginSegment(30);

        assertEquals(Continuation.REPLAN, replanner.continueLongPartialRoute(
                ORIGIN, target(8), longSegment(),
                PathStop.FRONTIER_TRUNCATED));

        // Ground closed, so only the endpoint guard can end this one.
        replanner.beginSegment(30);
        assertEquals(Continuation.STOP, replanner.continueLongPartialRoute(
                        ORIGIN, target(8), longSegment(),
                        PathStop.FRONTIER_TRUNCATED),
                "the same partial endpoint must never be submitted twice");
    }

    /** A follower standing eight blocks out having walked thirty to get there. */
    private static PartialRouteReplanner radiusCase() {
        PartialRouteReplanner replanner = new PartialRouteReplanner();
        replanner.reset(40);
        replanner.beginSegment(30);
        return replanner;
    }

    private static Point target(double distance) {
        return new Vec(distance, 1, 0);
    }

    private static List<PathNode> longSegment() {
        return route(-30, 0);
    }

    private static List<PathNode> route(int startX, int endX) {
        return List.of(
                new PathNode(startX + 0.5, 1, 0.5,
                        PathNode.Movement.WALK, startX, 1, 0),
                new PathNode(endX + 0.5, 1, 0.5,
                        PathNode.Movement.WALK, endX, 1, 0));
    }

    private static void wall(TestWorld world, int edge) {
        for (int offset = -edge; offset <= edge; offset++) {
            for (int y = 1; y <= 4; y++) {
                world.set(edge, y, offset, Block.STONE);
                world.set(-edge, y, offset, Block.STONE);
                world.set(offset, y, edge, Block.STONE);
                world.set(offset, y, -edge, Block.STONE);
            }
        }
    }

    /** A sealed ladder tower: the only edges off the floor go up. */
    private static TestWorld shaft(int topY) {
        TestWorld world = new TestWorld().floor(-2, 2, -1, 1, 0, Block.STONE);
        Block ladder = Block.LADDER.withProperty("facing", "west");
        for (int y = 1; y <= topY; y++) {
            world.set(1, y, 0, ladder);
            world.set(2, y, 0, Block.STONE);
        }
        for (int y = 1; y <= topY + 1; y++) {
            for (int x = -3; x <= 3; x++) {
                world.set(x, y, -2, Block.STONE);
                world.set(x, y, 2, Block.STONE);
            }
            for (int z = -2; z <= 2; z++) {
                world.set(-3, y, z, Block.STONE);
                world.set(3, y, z, Block.STONE);
            }
        }
        for (int x = -3; x <= 3; x++) {
            for (int z = -2; z <= 2; z++) {
                world.set(x, topY + 1, z, Block.STONE);
            }
        }
        return world;
    }

    private static PathResult climb(TestWorld world, Point start, Point target,
                                    double maxPathLength) {
        NavigationProfile base = BuiltinNavigationProfiles.forEntityType(
                EntityType.ZOMBIE);
        NavigationProfile climbing = base.withGroundCapabilities(
                base.groundCapabilities()
                        .withClimbables(ClimbableCapabilities.STANDARD)
                        .withHorizontalEdgeCost(true));
        NavigationRequest request = NavigationRequest.builder(
                        world, start, target, BOX, climbing)
                .maxPathLength(maxPathLength)
                .reachRange(1)
                .nodeSearchRange(maxPathLength)
                .build();
        return new EntityPathfinder().findPath(request, SearchControl.NONE);
    }

    private static PathResult search(TestWorld world, Point start, Point target,
                                     double maxPathLength, double visited) {
        NavigationRequest request = NavigationRequest.builder(world, start,
                        target, BOX,
                        BuiltinNavigationProfiles.forEntityType(
                                EntityType.ZOMBIE))
                .maxPathLength(maxPathLength)
                .reachRange(1)
                .maxVisitedMultiplier(visited)
                .nodeSearchRange(maxPathLength)
                .build();
        return new EntityPathfinder().findPath(request, SearchControl.NONE);
    }
}
