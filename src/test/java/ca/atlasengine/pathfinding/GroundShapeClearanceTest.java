package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.collision.CollisionUtils;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Partial block shapes and the physical room a route leaves for the box that
 * has to follow it. Waypoint heights come from the supporting block's real
 * collision shape, and level route segments must be sweepable by the entity's
 * complete bounding box.
 */
class GroundShapeClearanceTest {
    private static final BoundingBox STANDARD =
            new BoundingBox(0.6, 1.8, 0.6);
    private final DiscreteGroundPathfinder pathfinder =
            new DiscreteGroundPathfinder();

    /**
     * Vertical clearance is a walking constraint, not only a jump-arc one.
     * A two-block-high corridor capped at y=2 admits a 0.9-high mob and stops
     * a 1.8-high one at the mouth of the capped section.
     */
    @Test
    void lowEntityFitsUnderLowCeilingButTallEntityDoesNot() {
        TestWorld world = corridor();
        for (int z = 2; z <= 6; z++) {
            world.set(-1, 2, z, Block.STONE);
            world.set(0, 2, z, Block.STONE);
            world.set(1, 2, z, Block.STONE);
        }

        PathResult low = find(world, new Pos(0.5, 1, 0.5),
                new Pos(0.5, 1, 8.5), new BoundingBox(0.6, 0.9, 0.6), 32);
        PathResult tall = find(world, new Pos(0.5, 1, 0.5),
                new Pos(0.5, 1, 8.5), STANDARD, 32);

        assertTrue(low.found(), low::toString);
        assertFalse(tall.found(),
                () -> "tall entity crossed low ceiling: " + tall.nodes());
        assertTrue(tall.nodes().stream().noneMatch(node -> node.graphZ() >= 2),
                () -> "tall entity entered the capped section: "
                        + tall.nodes());
    }

    /**
     * A top slab fills the upper half of its cell, so the stand height above
     * it is a whole block; a bottom slab is a half step. Both occupy the same
     * graph node, so only the projected waypoint separates them.
     */
    @Test
    void topSlabIsAFullStepWhileBottomSlabIsAHalfStep() {
        Block topSlab = Block.STONE_SLAB.withProperty("type", "top");
        Block bottomSlab = Block.STONE_SLAB.withProperty("type", "bottom");

        PathResult top = find(narrowCorridor().set(1, 1, 0, topSlab),
                new Pos(0.5, 1, 0.5), new Pos(2.5, 1, 0.5), STANDARD, 32);
        PathResult bottom = find(narrowCorridor().set(1, 1, 0, bottomSlab),
                new Pos(0.5, 1, 0.5), new Pos(2.5, 1, 0.5), STANDARD, 32);

        assertTrue(top.found(), top::toString);
        PathNode onTopSlab = top.nodes().get(1);
        assertEquals(1, onTopSlab.graphX());
        assertEquals(2, onTopSlab.graphY());
        assertEquals(2.0, onTopSlab.y(), 1.0e-9,
                "a top slab raises the stand height by a whole block");
        assertEquals(PathNode.Movement.STEP_UP, onTopSlab.movement());

        assertTrue(bottom.found(), bottom::toString);
        PathNode onBottomSlab = bottom.nodes().get(1);
        assertEquals(1, onBottomSlab.graphX());
        assertEquals(2, onBottomSlab.graphY(),
                "both slab halves are the same graph node");
        assertEquals(1.5, onBottomSlab.y(), 1.0e-9,
                "a bottom slab raises the stand height by half a block");
    }

    /**
     * Carpet is 1/16 of a block tall and the graph uses that exact figure.
     * When carpet is the supporting block the waypoint sits on its collision
     * top; when carpet merely lies on a solid floor the cell stays walkable at
     * the floor's own top, because the stand height is read from the block
     * below the node rather than from the node's own contents.
     */
    @Test
    void carpetContributesItsExactSubBlockCollisionHeight() {
        double carpetTop = Block.RED_CARPET.collisionShape()
                .relativeEnd().y();
        assertEquals(0.0625, carpetTop, 1.0e-9);

        TestWorld supporting = new TestWorld()
                .floor(-2, 6, -1, 1, 0, Block.STONE)
                .set(0, 0, 0, Block.RED_CARPET);
        PathResult standing = find(supporting, new Pos(0.5, 1, 0.5),
                new Pos(4.5, 1, 0.5), STANDARD, 20);

        assertTrue(standing.found(), standing::toString);
        assertEquals(carpetTop, standing.nodes().getFirst().y(), 1.0e-9,
                "a supporting carpet contributes exactly its collision top");
        assertEquals(1, standing.nodes().getFirst().graphY(),
                "the sub-block height never becomes a graph coordinate");
        assertEquals(1.0, standing.nodes().get(1).y(), 1.0e-9,
                "the neighbouring full block is a whole-block stand height");

        TestWorld lying = new TestWorld().floor(-2, 8, -3, 3, 0, Block.STONE);
        for (int x = 1; x <= 4; x++) lying.set(x, 1, 0, Block.RED_CARPET);
        PathResult crossing = find(lying, new Pos(0.5, 1, 0.5),
                new Pos(5.5, 1, 0.5), STANDARD, 20);

        assertTrue(crossing.found(), crossing::toString);
        assertEquals(6, crossing.nodes().size(),
                "carpet is neither an obstacle nor a detour");
        assertTrue(crossing.nodes().stream().allMatch(node ->
                        node.graphZ() == 0 && node.graphY() == 1
                                && Math.abs(node.y() - 1.0) < 1.0e-9
                                && node.movement() == PathNode.Movement.WALK),
                () -> "a carpet lying on a solid floor changed the stand "
                        + "height or the route: " + crossing.nodes());
    }

    /**
     * A candle occupies a 2x2-pixel column. It must not price like a full
     * cube, must not force a detour, and must not multiply the search.
     */
    @Test
    void candleIsNeitherAWallNorASearchExplosion() {
        TestWorld world = new TestWorld()
                .floor(-12, 12, -12, 12, 0, Block.STONE)
                .set(0, 1, 3, Block.CANDLE);

        PathResult result = find(world, new Pos(0.5, 1, 0.5),
                new Pos(0.5, 1, 7.5), STANDARD, 32);

        assertTrue(result.found(), result::toString);
        assertEquals(8, result.nodes().size(),
                () -> "the candle bent the route: " + result.nodes());
        assertTrue(result.nodes().stream().allMatch(node ->
                        node.graphX() == 0 && node.graphY() == 1),
                () -> "the candle behaved like a wall: " + result.nodes());
        assertTrue(result.visitedNodes() < 100,
                () -> "small shape caused excessive search: "
                        + result.visitedNodes());
        assertTrue(result.visitedNodes() <= 8,
                () -> "a straight lane must expand one node per cell: "
                        + result.visitedNodes());
    }

    /**
     * Every level segment of a wall detour must be sweepable by the complete
     * bounding box, for every width the wall has to be cleared by.
     */
    @ParameterizedTest
    @ValueSource(doubles = {0.4, 0.8, 1.4, 1.8})
    void wallDetourKeepsSweptClearanceForEveryBoxWidth(double width) {
        TestWorld world = new TestWorld()
                .floor(-30, 30, -30, 30, 0, Block.STONE);
        for (int x = -8; x <= 8; x++) world.column(x, 5, 1, 3, Block.STONE);
        BoundingBox box = new BoundingBox(width, 1.8, width);

        PathResult result = find(world, new Pos(0.5, 1, 0.5),
                new Pos(0.5, 1, 12.5), box, 40);

        assertTrue(result.found(), result::toString);
        assertTrue(result.nodes().stream().anyMatch(node ->
                        Math.abs(node.x()) >= 9.5),
                () -> "path did not go around the wall end: " + result.nodes());
        assertLevelSegmentsClear(world, result, box);
    }

    /**
     * A one-wide, two-high tunnel is a hard physical filter: the complete
     * bounding box either fits it or the route may not use it, and a route
     * that does use it stays sweepable.
     */
    @ParameterizedTest
    @CsvSource({
            "0.80,1.80,true",
            "0.99,1.99,true",
            "1.01,1.80,false",
            "0.80,2.01,false"
    })
    void exactOneWideTwoHighGapRespectsFullBoundingBox(
            double width, double height, boolean expected) {
        TestWorld world = oneWideTwoHighTunnel();
        BoundingBox box = new BoundingBox(width, height, width);

        PathResult result = find(world, new Pos(0.5, 1, -4.5),
                new Pos(0.5, 1, 4.5), box, 20);

        assertEquals(expected, result.found(),
                () -> "unexpected path: " + result.nodes());
        if (expected) assertLevelSegmentsClear(world, result, box);
    }

    private static void assertLevelSegmentsClear(
            TestWorld world, PathResult result, BoundingBox box) {
        for (int i = 1; i < result.nodes().size(); i++) {
            PathNode previous = result.nodes().get(i - 1);
            PathNode current = result.nodes().get(i);
            if (Math.abs(previous.y() - current.y()) > 1.0e-4) continue;
            Vec start = previous.asVec().add(0, 1.0e-5, 0);
            Vec end = current.asVec().add(0, 1.0e-5, 0);
            var physics = CollisionUtils.handlePhysics(
                    world, box, start.asPos(), end.sub(start).asVec(),
                    null, false);
            assertFalse(physics.collisionX() || physics.collisionY()
                            || physics.collisionZ(),
                    () -> "blocked segment " + previous + " -> " + current);
        }
    }

    private PathResult find(TestWorld world, Pos start, Pos goal,
                            BoundingBox box, double maxLength) {
        return pathfinder.findPath(world, start, goal, box, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(maxLength).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
    }

    private static TestWorld flat() {
        return new TestWorld().floor(-24, 24, -24, 24, 0, Block.STONE);
    }

    private static TestWorld corridor() {
        TestWorld world = flat();
        for (int z = -24; z <= 24; z++) {
            world.column(-1, z, 1, 3, Block.STONE);
            world.column(1, z, 1, 3, Block.STONE);
        }
        return world;
    }

    private static TestWorld narrowCorridor() {
        TestWorld world = flat();
        for (int x = -24; x <= 24; x++) {
            world.column(x, -1, 1, 3, Block.STONE);
            world.column(x, 1, 1, 3, Block.STONE);
        }
        return world;
    }

    private static TestWorld oneWideTwoHighTunnel() {
        TestWorld world = new TestWorld()
                .floor(-10, 10, -10, 10, 0, Block.STONE);
        for (int z = -6; z <= 6; z++) {
            world.column(-1, z, 1, 3, Block.STONE);
            world.column(1, z, 1, 3, Block.STONE);
            world.set(0, 3, z, Block.STONE);
        }
        return world;
    }
}
