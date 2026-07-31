package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.EntityTraversalState;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.influence.InfluenceResult;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;
import net.minestom.server.entity.EntityType;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Behavioral fixtures for the integer ground-navigation graph.
 * Expected routes are stated explicitly rather than inferred from FOUND.
 */
class GroundNodePathfinderTest {
    private static final BoundingBox STANDARD =
            new BoundingBox(0.6, 1.8, 0.6);
    private final DiscreteGroundPathfinder pathfinder =
            new DiscreteGroundPathfinder();

    @Test
    void climbableRoutesAreExplicitlyOptInAndEmitVerticalClimbNodes() {
        TestWorld world = ladderTower(5);
        Pos start = new Pos(0.5, 1, 0.5);
        Pos top = new Pos(1.5, 5, 0.5);
        GroundCapabilities climbing = GroundCapabilities.STANDARD
                .withClimbables(ClimbableCapabilities.STANDARD);
        GroundCapabilities climbingWithoutLongFalls =
                GroundCapabilities.builder().maxStepHeight(1).maxFallDistance(1).allowDiagonal(true).horizontalEdgeCost(false).platformJump(PlatformJumpCapabilities.DISABLED).climbables(ClimbableCapabilities.STANDARD).build();

        PathResult baselineGround = find(world, start, top, STANDARD, 16);
        PathResult ascent = pathfinder.findPath(world, start, top, STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(16).reachRange(0).maxVisitedMultiplier(8).build(), climbing, SearchControl.NONE);
        PathResult descent = pathfinder.findPath(world, top, start, STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(16).reachRange(0).maxVisitedMultiplier(8).build(), climbingWithoutLongFalls, SearchControl.NONE);

        assertFalse(baselineGround.found(),
                "ordinary baseline ground profiles must not invent ladder edges");
        assertTrue(ascent.found(), ascent::toString);
        assertTrue(descent.found(), descent::toString);
        assertEquals(5, ascent.nodes().stream().filter(node ->
                node.movement() == PathNode.Movement.CLIMB).count());
        assertTrue(descent.nodes().stream().anyMatch(node ->
                node.movement() == PathNode.Movement.CLIMB));
    }

    @Test
    void ladderRouteOffsetsBoundingBoxOutsideTheClimbFace() {
        TestWorld world = ladderTower(5);
        GroundCapabilities climbing = GroundCapabilities.STANDARD
                .withClimbables(ClimbableCapabilities.STANDARD);

        PathResult result = pathfinder.findPath(world, new Pos(-0.5, 1, 0.5), new Pos(1.5, 5, 0.5), new BoundingBox(1.8, 1.8, 0.8), MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(16).reachRange(0).maxVisitedMultiplier(8).build(), climbing, SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        assertTrue(result.nodes().stream()
                        .filter(node -> node.graphX() == 1)
                        .allMatch(node -> node.x() < 0.1),
                () -> "mob was not offset outside ladder collision: "
                        + result.nodes());
    }

    @Test
    void completeMinecraftClimbableTagCanProduceOptInVerticalEdges() {
        List<String> ids = List.of("ladder", "vine", "scaffolding",
                "weeping_vines", "weeping_vines_plant",
                "twisting_vines", "twisting_vines_plant",
                "cave_vines", "cave_vines_plant");
        GroundCapabilities climbing = GroundCapabilities.STANDARD
                .withClimbables(ClimbableCapabilities.STANDARD);

        for (String id : ids) {
            Block source = assertDoesNotThrow(() -> Block.fromKey(id));
            assertNotNull(source, id);
            TestWorld world = new TestWorld().floor(
                    -2, 3, -1, 1, 0, Block.STONE);
            Block state = id.equals("ladder")
                    ? source.withProperty("facing", "west") : source;
            for (int y = 1; y <= 4; y++) world.set(1, y, 0, state);
            if (id.equals("ladder")) {
                for (int y = 1; y <= 4; y++) {
                    world.set(2, y, 0, Block.STONE);
                }
            }

            PathResult result = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(1.5, 5, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(16).reachRange(0).maxVisitedMultiplier(8).build(), climbing, SearchControl.NONE);

            assertTrue(BlockTraversalData.isClimbable(state), id);
            assertTrue(result.found(), () -> id + ": " + result);
            assertTrue(result.nodes().stream().anyMatch(node ->
                    node.movement() == PathNode.Movement.CLIMB), id);
        }
    }

    @Test
    void climbDirectionsAndInterruptedColumnsAreHardConstraints() {
        TestWorld world = ladderTower(5);
        Pos bottom = new Pos(0.5, 1, 0.5);
        Pos top = new Pos(1.5, 5, 0.5);
        GroundCapabilities ascentOnly = GroundCapabilities.STANDARD
                .withClimbables(ClimbableCapabilities.builder().enabled(true).allowAscending(true).allowDescending(false).verticalCostMultiplier(1).build());
        GroundCapabilities ascentOnlyWithoutLongFalls =
                GroundCapabilities.builder().maxStepHeight(1).maxFallDistance(1).allowDiagonal(true).horizontalEdgeCost(false).platformJump(PlatformJumpCapabilities.DISABLED).climbables(ClimbableCapabilities.builder().enabled(true).allowAscending(true).allowDescending(false).verticalCostMultiplier(1).build()).build();
        GroundCapabilities descentOnly = GroundCapabilities.builder().maxStepHeight(1).maxFallDistance(1).allowDiagonal(true).horizontalEdgeCost(false).platformJump(PlatformJumpCapabilities.DISABLED).climbables(ClimbableCapabilities.builder().enabled(true).allowAscending(false).allowDescending(true).verticalCostMultiplier(1).build()).build();

        PathResult up = pathfinder.findPath(world, bottom, top, STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(16).reachRange(0).maxVisitedMultiplier(8).build(), ascentOnly, SearchControl.NONE);
        PathResult downRejected = pathfinder.findPath(world, top, bottom, STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(16).reachRange(0).maxVisitedMultiplier(8).build(), ascentOnlyWithoutLongFalls, SearchControl.NONE);
        PathResult down = pathfinder.findPath(world, top, bottom, STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(16).reachRange(0).maxVisitedMultiplier(8).build(), descentOnly, SearchControl.NONE);
        TestWorld broken = ladderTower(5).set(1, 3, 0, Block.AIR);
        PathResult gap = pathfinder.findPath(broken, bottom, top, STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(16).reachRange(0).maxVisitedMultiplier(8).build(), ascentOnly, SearchControl.NONE);

        assertTrue(up.found(), up::toString);
        assertFalse(downRejected.found());
        assertTrue(down.found(), down::toString);
        assertFalse(gap.found(),
                "a climb route must not bridge a missing climbable cell");
    }

    @Test
    void optInPlatformJumpCrossesOneCellGap() {
        TestWorld world = platformGap(2);
        GroundCapabilities jumper = GroundCapabilities.STANDARD
                .withPlatformJump(PlatformJumpCapabilities.builder().maxHorizontalDistance(2).maxRise(0).maxDrop(0).apexClearance(1).build());

        PathResult ordinary = find(world, new Pos(0.5, 1, 0.5),
                new Pos(2.5, 1, 0.5), STANDARD, 5);
        PathResult jumping = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(2.5, 1, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(5).reachRange(0).maxVisitedMultiplier(8).build(), jumper, SearchControl.NONE);

        assertFalse(ordinary.found());
        assertTrue(jumping.found(), jumping::toString);
        assertEquals(2, jumping.nodes().size());
        assertEquals(PathNode.Movement.JUMP,
                jumping.nodes().getLast().movement());

        PathResult arcBudgetRejected = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(2.5, 1, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(2.5).reachRange(0).maxVisitedMultiplier(8).build(), jumper, SearchControl.NONE);
        assertFalse(arcBudgetRejected.found(),
                "maxPathLength must charge the jump arc, not only its chord");
    }

    @Test
    void configuredJumpDistanceIsAnInclusiveHardBoundary() {
        TestWorld world = platformGap(3);
        GroundCapabilities tooShort = GroundCapabilities.STANDARD
                .withPlatformJump(PlatformJumpCapabilities.builder().maxHorizontalDistance(2).maxRise(0).maxDrop(0).apexClearance(1).build());
        GroundCapabilities exact = GroundCapabilities.STANDARD
                .withPlatformJump(PlatformJumpCapabilities.builder().maxHorizontalDistance(3).maxRise(0).maxDrop(0).apexClearance(1).build());

        PathResult rejected = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(3.5, 1, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(6).reachRange(0).maxVisitedMultiplier(8).build(), tooShort, SearchControl.NONE);
        PathResult accepted = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(3.5, 1, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(6).reachRange(0).maxVisitedMultiplier(8).build(), exact, SearchControl.NONE);

        assertFalse(rejected.found());
        assertTrue(accepted.found(), accepted::toString);
        assertEquals(PathNode.Movement.JUMP,
                accepted.nodes().getLast().movement());
        assertEquals(3, accepted.nodes().getLast().graphX());
    }

    @Test
    void configuredRiseAndDropAreInclusiveHardBoundaries() {
        TestWorld rise = platformGapWithLandingSupportY(2, 1);
        TestWorld drop = platformGapWithLandingSupportY(2, -1);
        GroundCapabilities levelOnly = GroundCapabilities.STANDARD
                .withPlatformJump(PlatformJumpCapabilities.builder().maxHorizontalDistance(2).maxRise(0.99).maxDrop(0.99).apexClearance(2).build());
        GroundCapabilities oneBlock = GroundCapabilities.STANDARD
                .withPlatformJump(PlatformJumpCapabilities.builder().maxHorizontalDistance(2).maxRise(1).maxDrop(1).apexClearance(2).build());

        PathResult riseRejected = jumpPath(
                rise, new Pos(0.5, 1, 0.5),
                new Pos(2.5, 2, 0.5), STANDARD, levelOnly);
        PathResult riseAccepted = jumpPath(
                rise, new Pos(0.5, 1, 0.5),
                new Pos(2.5, 2, 0.5), STANDARD, oneBlock);
        PathResult dropRejected = jumpPath(
                drop, new Pos(0.5, 1, 0.5),
                new Pos(2.5, 0, 0.5), STANDARD, levelOnly);
        PathResult dropAccepted = jumpPath(
                drop, new Pos(0.5, 1, 0.5),
                new Pos(2.5, 0, 0.5), STANDARD, oneBlock);

        assertFalse(riseRejected.found());
        assertTrue(riseAccepted.found(), riseAccepted::toString);
        assertEquals(2, riseAccepted.nodes().getLast().graphY());
        assertFalse(dropRejected.found());
        assertTrue(dropAccepted.found(), dropAccepted::toString);
        assertEquals(0, dropAccepted.nodes().getLast().graphY());
    }

    @Test
    void partialJumpPathEndsOnLastReachablePlatform() {
        TestWorld world = platformGap(2)
                .set(6, 0, 0, Block.STONE);
        for (int x = 3; x <= 8; x++) {
            world.column(x, -1, 0, 4, Block.STONE);
            world.column(x, 1, 0, 4, Block.STONE);
        }
        GroundCapabilities jumper = GroundCapabilities.STANDARD
                .withPlatformJump(PlatformJumpCapabilities.builder().maxHorizontalDistance(2).maxRise(0).maxDrop(0).apexClearance(1).build());

        PathResult result = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(6.5, 1, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(12).reachRange(0).maxVisitedMultiplier(8).build(), jumper, SearchControl.NONE);

        assertEquals(PathStatus.PARTIAL, result.status());
        assertEquals(2, result.nodes().getLast().graphX());
        assertEquals(1, result.nodes().getLast().graphY());
        assertEquals(PathNode.Movement.JUMP,
                result.nodes().getLast().movement());
        assertEquals(2, result.nodes().size(),
                "a jump is one atomic supported-to-supported edge");
    }

    @Test
    void jumpCandidateChecksScaleLinearlyWithDistance() {
        TestWorld world = new TestWorld().set(0, 0, 0, Block.STONE);
        GroundCapabilities jumper = GroundCapabilities.STANDARD
                .withPlatformJump(PlatformJumpCapabilities.builder().maxHorizontalDistance(32).maxRise(0).maxDrop(0).apexClearance(1).build());

        PathResult result = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(100.5, 1, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(64).reachRange(0).maxVisitedMultiplier(8).build(), jumper, SearchControl.NONE);

        assertEquals(PathStatus.PARTIAL, result.status());
        assertEquals(1, result.visitedNodes());
        assertTrue(result.examinedNeighbors() <= 8 + 4 * 31,
                () -> "jump ray work exceeded its linear bound: "
                        + result.examinedNeighbors());
    }

    @Test
    void platformJumpArcRejectsLowCeiling() {
        TestWorld clear = platformGap(2);
        TestWorld ceiling = platformGap(2)
                .set(1, 3, 0, Block.STONE);
        GroundCapabilities jumper = GroundCapabilities.STANDARD
                .withPlatformJump(PlatformJumpCapabilities.builder().maxHorizontalDistance(2).maxRise(0).maxDrop(0).apexClearance(1).build());

        PathResult accepted = pathfinder.findPath(clear, new Pos(0.5, 1, 0.5), new Pos(2.5, 1, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(5).reachRange(0).maxVisitedMultiplier(8).build(), jumper, SearchControl.NONE);
        PathResult rejected = pathfinder.findPath(ceiling, new Pos(0.5, 1, 0.5), new Pos(2.5, 1, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(5).reachRange(0).maxVisitedMultiplier(8).build(), jumper, SearchControl.NONE);

        assertTrue(accepted.found(), accepted::toString);
        assertFalse(rejected.found(),
                "the complete entity bounding box must clear the jump arc");
    }

    @Test
    void platformJumpCapabilityDoesNotReplaceOrdinaryShapeEdges() {
        TestWorld world = new TestWorld()
                .floor(-2, 6, -1, 1, 0, Block.STONE)
                .set(1, 1, 0, Block.WHITE_CARPET)
                .set(2, 1, 0, Block.STONE_SLAB
                        .withProperty("type", "bottom"))
                .set(3, 1, 0, Block.STONE_STAIRS
                        .withProperty("facing", "east")
                        .withProperty("half", "bottom"));
        GroundCapabilities jumper = GroundCapabilities.STANDARD
                .withPlatformJump(PlatformJumpCapabilities.builder().maxHorizontalDistance(4).maxRise(1).maxDrop(1).apexClearance(1).build());

        PathResult result = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(5.5, 1, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(10).reachRange(0).maxVisitedMultiplier(8).build(), jumper, SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        assertTrue(result.nodes().stream().noneMatch(node ->
                        node.movement() == PathNode.Movement.JUMP),
                () -> "ordinary partial shapes became jump edges: "
                        + result.nodes());
        for (int index = 1; index < result.nodes().size(); index++) {
            assertTrue(Math.abs(result.nodes().get(index).graphX()
                    - result.nodes().get(index - 1).graphX()) <= 1);
        }
    }

    @Test
    void physicalSlabHeightDoesNotChangeHorizontalGraphCost() {
        TestWorld world = new TestWorld();
        for (int x = -2; x <= 6; x++) {
            world.set(x, 0, 0, (x & 1) == 0
                    ? Block.STONE
                    : Block.STONE_SLAB.withProperty("type", "bottom"));
        }

        PathResult result = find(world, new Pos(0.5, 1, 0.5),
                new Pos(4.5, 1, 0.5), STANDARD, 8);

        assertTrue(result.found());
        assertEquals(5, result.nodes().size());
        for (int x = 0; x <= 4; x++) {
            assertEquals(x, result.nodes().get(x).blockX());
            assertEquals(0.5 + 0.5 * ((x + 1) & 1),
                    result.nodes().get(x).y(), 1.0e-9);
        }
        assertEquals(4 + 4 * (Math.sqrt(1.25) - 1),
                physicalLength(result), 1.0e-9,
                "projection changes follower distance, never integer A* edge cost");
    }

    @Test
    void stickyHoneySourceCannotInitiateObstacleJump() {
        TestWorld normal = jumpCorridor(Block.STONE);
        TestWorld honey = jumpCorridor(Block.HONEY_BLOCK);

        PathResult ordinary = find(normal, new Pos(1.5, 1, 0.5),
                new Pos(3.5, 1, 0.5), STANDARD, 6);
        PathResult sticky = find(honey, new Pos(1.5, 1, 0.5),
                new Pos(3.5, 1, 0.5), STANDARD, 6);

        assertTrue(ordinary.found(), () -> ordinary.nodes().toString());
        assertTrue(ordinary.nodes().stream().anyMatch(node ->
                node.blockX() == 2 && node.blockY() == 2));
        assertFalse(sticky.found(),
                "sticky honey suppresses upward departure steps");
    }

    @Test
    void pointEightWideOnePointEightTallMobFitsOneByTwoOpening() {
        TestWorld world = new TestWorld().floor(-4, 9, -12, 12, 0, Block.STONE);
        for (int z = -12; z <= 12; z++) {
            if (z == 0) continue;
            world.column(3, z, 1, 3, Block.STONE);
        }
        BoundingBox fits = new BoundingBox(0.8, 1.8, 0.8);
        BoundingBox tooWide = new BoundingBox(1.01, 1.8, 1.01);

        PathResult narrow = find(world, new Pos(0.5, 1, 0.5),
                new Pos(6.5, 1, 0.5), fits, 9);
        PathResult wide = find(world, new Pos(0.5, 1, 0.5),
                new Pos(6.5, 1, 0.5), tooWide, 9);

        assertTrue(narrow.found(), () -> narrow.nodes().toString());
        assertEquals(7, narrow.nodes().size());
        assertTrue(narrow.nodes().stream().allMatch(node -> node.blockZ() == 0));
        assertFalse(wide.found(),
                "floor(width+1)=2 samples the adjacent occupied wall cell");
    }

    @Test
    void nonFloatingWalkerKeepsSingleSurfaceWaterCandidate() {
        TestWorld world = new TestWorld()
                .floor(-2, 4, -1, 1, 0, Block.STONE)
                .set(1, 1, 0, Block.WATER);
        MobTraversalProfile profile = MobTraversalProfile.builder("water_walker")
                .malus(TerrainType.WATER, 0)
                .build();

        PathResult result = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(1.5, 1, 0.5), STANDARD, profile, GroundSearchLimits.builder().maxPathLength(8).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        assertEquals(1, result.nodes().getLast().graphX());
        assertEquals(1, result.nodes().getLast().graphY(),
                "if y-1 is already non-water, the initial WATER node remains "
                        + "the accepted neighbor");
    }

    @Test
    void fallDistanceBoundaryIsInclusiveAndNextBlockIsRejected() {
        TestWorld world = new TestWorld();
        world.set(0, 3, 0, Block.STONE);
        for (int x = 1; x <= 4; x++) world.set(x, 1, 0, Block.STONE);
        for (int x = -1; x <= 5; x++) {
            world.column(x, -1, 1, 6, Block.STONE);
            world.column(x, 1, 1, 6, Block.STONE);
        }

        PathResult exactBoundary = pathfinder.findPath(world, new Pos(0.5, 4, 0.5), new Pos(3.5, 2, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(8).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.builder().maxStepHeight(1).maxFallDistance(2).allowDiagonal(true).build(), SearchControl.NONE);
        PathResult oneTooFar = pathfinder.findPath(world, new Pos(0.5, 4, 0.5), new Pos(3.5, 2, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(8).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.builder().maxStepHeight(1).maxFallDistance(1).allowDiagonal(true).build(), SearchControl.NONE);

        assertTrue(exactBoundary.found(), () -> exactBoundary.nodes().toString());
        assertEquals(2, exactBoundary.nodes().get(1).blockY());
        assertFalse(oneTooFar.found(),
                "baseline checks sourceY-currentY > maxFall before classification");
    }

    @Test
    void subBlockStepCapabilityStillGetsOneRecursiveJumpNode() {
        TestWorld world = jumpCorridor(Block.STONE);
        PathResult result = pathfinder.findPath(world, new Pos(1.5, 1, 0.5), new Pos(3.5, 1, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(6).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.builder().maxStepHeight(0.5).maxFallDistance(5).allowDiagonal(true).build(), SearchControl.NONE);

        assertTrue(result.found(), () -> result.nodes().toString());
        assertTrue(result.nodes().stream().anyMatch(node ->
                node.blockX() == 2 && node.blockY() == 2),
                "jump recursion budget is floor(max(1,maxUpStep))");
    }

    @Test
    void explicitAirborneStateDescendsToFirstNonPathfindableSupport() {
        TestWorld world = new TestWorld().floor(-3, 5, -2, 2, 0, Block.STONE);
        NavigationProfile zombie =
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);

        PathResult airborne = dispatch(world, new Pos(0.5, 4.2, 0.5),
                new Pos(3.5, 1, 0.5), zombie,
                new EntityTraversalState(false, false, java.util.Set.of(), -64));
        PathResult claimedGrounded = dispatch(world, new Pos(0.5, 4.2, 0.5),
                new Pos(3.5, 1, 0.5), zombie,
                EntityTraversalState.GROUNDED);

        assertTrue(airborne.found(), airborne::toString);
        assertEquals(1, airborne.nodes().getFirst().graphY(),
                "airborne getStart scans down to one node above solid support");
        assertEquals(4, claimedGrounded.nodes().getFirst().graphY(),
                "on-ground branch is floor(entityY + 0.5), even if caller lies");
        assertEquals(4, claimedGrounded.nodes().getFirst().y(), 1.0e-9,
                "an airborne waypoint whose supporting cell is air retains "
                        + "its graph Y");
    }

    @Test
    void explicitWaterAndFluidStandingStatesSelectColumnSurface() {
        TestWorld water = new TestWorld();
        TestWorld lava = new TestWorld();
        for (int x = -1; x <= 4; x++) {
            for (int y = 0; y <= 3; y++) {
                water.set(x, y, 0, Block.WATER);
                lava.set(x, y, 0, Block.LAVA);
            }
        }
        EntityTraversalState swimming = new EntityTraversalState(
                false, true, java.util.Set.of(), -64);
        EntityTraversalState lavaStanding = new EntityTraversalState(
                false, false, java.util.Set.of(Block.LAVA), -64);

        PathResult chicken = dispatch(
                water, new Pos(0.5, 1.2, 0.5), new Pos(3.5, 3, 0.5),
                BuiltinNavigationProfiles.forEntityType(EntityType.CHICKEN),
                swimming);
        PathResult strider = dispatch(
                lava, new Pos(0.5, 1.2, 0.5), new Pos(3.5, 3, 0.5),
                BuiltinNavigationProfiles.forEntityType(EntityType.STRIDER),
                lavaStanding);

        assertTrue(chicken.found(), chicken::toString);
        assertEquals(3, chicken.nodes().getFirst().graphY());
        assertTrue(strider.found(), strider::toString);
        assertEquals(3, strider.nodes().getFirst().graphY(),
                "canStandOnFluid climbs through the standable fluid column");
    }

    @Test
    void groundDispatcherNormalizesAirAndSolidTargetsToSurface() {
        TestWorld world = new TestWorld().floor(-3, 7, -2, 2, 0, Block.STONE);
        world.set(2, 1, 0, Block.STONE);
        world.column(3, 0, 1, 2, Block.STONE);
        world.column(4, 0, 1, 3, Block.STONE);
        NavigationProfile zombie =
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);

        PathResult highAir = dispatch(world, new Pos(0.5, 1, 0.5),
                new Pos(5.5, 12, 0.5), zombie, EntityTraversalState.GROUNDED);
        PathResult insideSolid = dispatch(world, new Pos(0.5, 1, 0.5),
                new Pos(4.5, 2, 0.5), zombie, EntityTraversalState.GROUNDED);

        assertTrue(highAir.found(), highAir::toString);
        assertEquals(1, highAir.nodes().getLast().graphY(),
                "air target scans down to the first surface");
        assertTrue(insideSolid.found(), insideSolid::toString);
        assertEquals(4, insideSolid.nodes().getLast().graphY(),
                "solid target scans upward to the first non-solid block");
    }

    @Test
    void reachedSearchReturnsClosestDiscoveredTargetNode() {
        TestWorld world = new TestWorld().floor(
                -3, 5, -3, 5, 0, Block.STONE);
        NavigationInfluence expensiveTarget = (blocks, point, box) ->
                point.blockX() == 1 && point.blockY() == 1
                        && point.blockZ() == 1
                        ? InfluenceResult.penalty(100, "target")
                        : InfluenceResult.NONE;
        NavigationProfile zombie =
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);

        PathResult result = new EntityPathfinder().findPath(
                NavigationRequest.builder(
                                world, new Pos(0.5, 1, 0.5),
                                new Pos(1.5, 1, 1.5), STANDARD, zombie)
                        .maxPathLength(12)
                        .reachRange(1)
                        .influences(List.of(expensiveTarget))
                        .build(),
                SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        PathNode end = result.nodes().getLast();
        assertEquals(1, end.graphX());
        assertEquals(1, end.graphY());
        assertEquals(1, end.graphZ(),
                "reach-range termination reconstructs the closest discovered "
                        + "target node rather than the popped cardinal");
    }

    @Test
    void adjustedBudgetUsesDiscoveredPartialAndCanReturnStartOnlyPath() {
        TestWorld world = new TestWorld().floor(
                -3, 8, -3, 3, 0, Block.STONE);
        double oneExpansionBudget = 2.0 / (16 * 16);
        PathResult discovered = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(5.5, 1, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(12).reachRange(0).maxVisitedMultiplier(oneExpansionBudget).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
        PathResult startOnly = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(5.5, 1, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(12).reachRange(0).maxVisitedMultiplier(1.0 / (16 * 16)).build(), GroundCapabilities.STANDARD, SearchControl.NONE);

        assertEquals(PathStatus.PARTIAL, discovered.status());
        assertEquals(1, discovered.visitedNodes());
        assertEquals(1, discovered.nodes().getLast().graphX(),
                "closest endpoint updates when the east neighbor is discovered");
        assertEquals(PathStatus.PARTIAL, startOnly.status());
        assertEquals(0, startOnly.visitedNodes());
        assertEquals(1, startOnly.nodes().size(),
                "a valid evaluator start produces a one-node unreached path");
        assertEquals(0, startOnly.nodes().getFirst().graphX());
    }

    @Test
    void invalidStartUsesActualAsymmetricBoundingBoxCorners() {
        TestWorld world = new TestWorld().floor(
                -2, 7, -2, 5, 0, Block.STONE);
        world.set(0, 1, 0, Block.STONE);
        world.set(0, 1, 1, Block.STONE);
        world.set(1, 1, 0, Block.STONE);
        BoundingBox offsetBox = new BoundingBox(
                0.6, 1.8, 0.6, new Vec(0.3, 0, 0.3));

        PathResult result = pathfinder.findPath(world, new Pos(0.8, 1, 0.8), new Pos(4.5, 1, 1.5), offsetBox, MobTraversalProfile.DEFAULT, GroundSearchLimits.ofPathLength(12), GroundCapabilities.STANDARD, SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        assertEquals(1, result.nodes().getFirst().graphX());
        assertEquals(1, result.nodes().getFirst().graphZ(),
                "corner recovery uses relativeStart/relativeEnd, not an "
                        + "assumed centered half-width");
    }

    @Test
    void rectangularBoundingBoxUsesIndependentWidthAndDepthAnchors() {
        TestWorld world = new TestWorld().floor(
                -3, 8, -3, 3, 0, Block.STONE);
        BoundingBox rectangular = new BoundingBox(1.8, 1.8, 0.8);

        PathResult result = find(world, new Pos(1, 1, 0.5),
                new Pos(5, 1, 0.5), rectangular, 12);

        assertTrue(result.found(), result::toString);
        assertTrue(result.nodes().stream().allMatch(node -> node.x() % 1 == 0),
                result.nodes()::toString);
        assertTrue(result.nodes().stream().allMatch(node -> node.z() == 0.5),
                () -> "depth anchor was incorrectly shifted by width: "
                        + result.nodes());
    }

    @Test
    void disabledDiagonalCapabilityProducesOnlyCardinalEdges() {
        TestWorld world = new TestWorld().floor(
                -3, 8, -3, 8, 0, Block.STONE);

        PathResult result = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(4.5, 1, 4.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.ofPathLength(20), GroundCapabilities.builder().maxStepHeight(1).maxFallDistance(5).allowDiagonal(false).build(), SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        for (int index = 1; index < result.nodes().size(); index++) {
            PathNode previous = result.nodes().get(index - 1);
            PathNode current = result.nodes().get(index);
            int changedAxes = Math.abs(current.graphX() - previous.graphX())
                    + Math.abs(current.graphZ() - previous.graphZ());
            assertEquals(1, changedAxes, result.nodes()::toString);
        }
    }

    private PathResult find(TestWorld world, Pos start, Pos goal,
                            BoundingBox box, double maxLength) {
        return pathfinder.findPath(world, start, goal, box, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(maxLength).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
    }

    private PathResult jumpPath(
            TestWorld world, Pos start, Pos goal, BoundingBox box,
            GroundCapabilities capabilities) {
        return pathfinder.findPath(world, start, goal, box, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(12).reachRange(0).maxVisitedMultiplier(8).build(), capabilities, SearchControl.NONE);
    }

    private static PathResult dispatch(
            TestWorld world, Pos start, Pos target,
            NavigationProfile profile, EntityTraversalState state) {
        return new EntityPathfinder().findPath(
                NavigationRequest.builder(
                                world, start, target, STANDARD, profile)
                        .maxPathLength(32)
                        .maxVisitedMultiplier(8)
                        .entityState(state)
                        .build(),
                SearchControl.NONE);
    }

    private static TestWorld jumpCorridor(Block sourceSupport) {
        TestWorld world = new TestWorld().floor(-2, 6, -1, 1, 0, Block.STONE);
        world.set(1, 0, 0, sourceSupport);
        world.set(2, 1, 0, Block.STONE);
        for (int x = -2; x <= 6; x++) {
            world.column(x, -1, 1, 4, Block.STONE);
            world.column(x, 1, 1, 4, Block.STONE);
        }
        return world;
    }

    private static TestWorld ladderTower(int topY) {
        TestWorld world = new TestWorld().floor(-2, 3, -1, 1, 0, Block.STONE);
        Block ladder = Block.LADDER.withProperty("facing", "west");
        for (int y = 1; y <= topY; y++) {
            world.set(1, y, 0, ladder);
            world.set(2, y, 0, Block.STONE);
        }
        return world;
    }

    private static TestWorld platformGap(int landingX) {
        TestWorld world = new TestWorld()
                .set(0, 0, 0, Block.STONE)
                .set(landingX, 0, 0, Block.STONE);
        for (int x = -2; x <= landingX + 2; x++) {
            world.column(x, -1, 0, 4, Block.STONE);
            world.column(x, 1, 0, 4, Block.STONE);
        }
        return world;
    }

    private static TestWorld platformGapWithLandingSupportY(
            int landingX, int landingSupportY) {
        TestWorld world = new TestWorld()
                .set(0, 0, 0, Block.STONE)
                .set(landingX, landingSupportY, 0, Block.STONE);
        for (int x = -2; x <= landingX + 2; x++) {
            world.column(x, -1, -2, 4, Block.STONE);
            world.column(x, 1, -2, 4, Block.STONE);
        }
        return world;
    }

    private static double physicalLength(PathResult result) {
        double length = 0;
        for (int i = 1; i < result.nodes().size(); i++) {
            length += result.nodes().get(i - 1).asVec()
                    .distance(result.nodes().get(i).asVec());
        }
        return length;
    }
}
