package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.search.EntityTraversalState;
import ca.atlasengine.pathfinding.profile.GroundCapabilities;
import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.result.PathStatus;
import ca.atlasengine.pathfinding.search.SearchControl;
import ca.atlasengine.pathfinding.search.SpatialPathfinder;
import ca.atlasengine.pathfinding.TestWorld;
import ca.atlasengine.pathfinding.influence.InfluenceResult;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.Locale;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SpatialPathfinderTest {
    private static final BoundingBox SMALL = new BoundingBox(0.6, 0.6, 0.6);
    private final SpatialPathfinder pathfinder = new SpatialPathfinder();

    @Test
    void registrySelectsEachBuiltInNavigationFamily() {
        assertEquals(NavigationMode.GROUND,
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE).mode());
        assertEquals(NavigationMode.WALL_CLIMBER,
                BuiltinNavigationProfiles.forEntityType(EntityType.SPIDER).mode());
        assertEquals(NavigationMode.WATER,
                BuiltinNavigationProfiles.forEntityType(EntityType.COD).mode());
        assertEquals(NavigationMode.AMPHIBIOUS,
                BuiltinNavigationProfiles.forEntityType(EntityType.FROG).mode());
        assertEquals(NavigationMode.FLYING,
                BuiltinNavigationProfiles.forEntityType(EntityType.BEE).mode());
        assertEquals(NavigationMode.FLYING,
                BuiltinNavigationProfiles.forEntityType(EntityType.PARROT).mode());
        assertEquals(NavigationMode.WATER,
                BuiltinNavigationProfiles.forEntityType(EntityType.NAUTILUS).mode());
        assertEquals(NavigationMode.GROUND,
                BuiltinNavigationProfiles.forEntityType(EntityType.BREEZE).mode());
        assertTrue(BuiltinNavigationProfiles.forEntityType(EntityType.DOLPHIN).allowBreaching());
        assertFalse(BuiltinNavigationProfiles.forEntityType(EntityType.COD).allowBreaching());
    }

    @Test
    void swimmerUsesAxialVerticalRoute() {
        TestWorld world = waterBox(-1, 5, -1, 3, -2, 2);
        PathResult result = find(world, new Vec(0.5, 0.5, 0.5),
                new Vec(4.5, 2.5, 0.5), waterProfile(false), SMALL, 30);

        assertTrue(result.found(), result.status() + " " + result.nodes()
                + " visited=" + result.visitedNodes());
        assertEquals(PathNode.Movement.SWIM, result.nodes().getLast().movement());
        assertTrue(result.nodes().stream().anyMatch(node -> node.blockY() > 0));
        assertFalse(hasMultiAxisStep(result),
                "baseline does not generate vertical-horizontal swim diagonals");
    }

    @Test
    void swimmerDetoursAroundDryCellAndNeverEntersIt() {
        TestWorld world = waterBox(0, 4, 0, 0, -1, 1);
        world.set(2, 0, 0, Block.AIR);

        PathResult result = find(world, new Vec(0.5, 0.5, 0.5),
                new Vec(4.5, 0.5, 0.5), waterProfile(false), SMALL, 20);

        assertTrue(result.found(), () -> result.status() + " " + result.nodes());
        assertTrue(result.nodes().stream().anyMatch(node -> node.blockZ() != 0));
        assertFalse(result.nodes().stream().anyMatch(node ->
                node.blockX() == 2 && node.blockY() == 0 && node.blockZ() == 0));
    }

    @Test
    void swimmerCannotCrossFullDryBarrier() {
        TestWorld world = waterBox(0, 4, 0, 0, -1, 1);
        for (int z = -1; z <= 1; z++) world.set(2, 0, z, Block.AIR);

        PathResult result = find(world, new Vec(0.5, 0.5, 0.5),
                new Vec(4.5, 0.5, 0.5), waterProfile(false), SMALL, 12);

        assertFalse(result.found());
        assertTrue(result.nodes().stream().allMatch(node -> node.blockX() < 2));
    }

    @Test
    void swimmerAcceptsWaterloggedSlabButRejectsWaterloggedStair() {
        TestWorld slabWorld = waterBox(0, 4, 0, 0, 0, 0)
                .set(2, 0, 0, Block.OAK_SLAB
                        .withProperty("waterlogged", "true"));
        TestWorld stairWorld = waterBox(0, 4, 0, 0, 0, 0)
                .set(2, 0, 0, Block.OAK_STAIRS
                        .withProperty("waterlogged", "true"));

        PathResult slab = find(slabWorld, new Vec(0.5, 0.1, 0.5),
                new Vec(4.5, 0.1, 0.5), waterProfile(false), SMALL, 12);
        PathResult stair = find(stairWorld, new Vec(0.5, 0.1, 0.5),
                new Vec(4.5, 0.1, 0.5), waterProfile(false), SMALL, 12);

        assertTrue(slab.found(), slab::toString);
        assertTrue(slab.nodes().stream().anyMatch(node ->
                node.graphX() == 2 && node.graphY() == 0
                        && node.graphZ() == 0));
        assertFalse(stair.found(), stair::toString);
        assertTrue(stair.nodes().stream().allMatch(node -> node.graphX() < 2));
    }

    @Test
    void swimmerWaypointsProjectFromIntegerVolumeAnchorForEachWidth() {
        TestWorld world = waterBox(-2, 7, -1, 3, -3, 4);
        BoundingBox wide = new BoundingBox(1.8, 1.8, 1.8);
        Vec start = new Vec(1, 0.5, 1);
        Vec target = new Vec(4.5, 0.5, 1);

        PathResult small = find(
                world, start, target, waterProfile(false), SMALL, 12);
        PathResult large = find(
                world, start, target, waterProfile(false), wide, 12);

        assertTrue(small.found(), small::toString);
        assertTrue(large.found(), large::toString);
        assertTrue(small.nodes().stream().allMatch(node ->
                Math.abs(node.x() - (node.graphX() + 0.5)) < 1.0e-9
                        && Math.abs(node.z() - (node.graphZ() + 0.5))
                        < 1.0e-9));
        assertTrue(large.nodes().stream().allMatch(node ->
                Math.abs(node.x() - (node.graphX() + 1.0)) < 1.0e-9
                        && Math.abs(node.z() - (node.graphZ() + 1.0))
                        < 1.0e-9));
    }

    @Test
    void spatialVolumeAndWaypointsUseIndependentWidthAndDepth() {
        TestWorld world = waterBox(-2, 7, -1, 2, -3, 5);
        BoundingBox rectangular = new BoundingBox(1.8, 1.0, 0.6);

        PathResult result = find(world, new Vec(1, 0.5, 1),
                new Vec(4.5, 0.5, 1), waterProfile(false), rectangular, 12);

        assertTrue(result.found(), result::toString);
        assertTrue(result.nodes().stream().allMatch(node ->
                        Math.abs(node.x() - (node.graphX() + 1.0)) < 1.0e-9
                                && Math.abs(node.z()
                                - (node.graphZ() + 0.5)) < 1.0e-9),
                "spatial projection must not substitute width for depth");

        TestWorld narrowDepthGap = waterBox(-1, 5, 0, 2, 0, 0);
        PathResult throughGap = find(narrowDepthGap,
                new Vec(0.5, 0.5, 0.5), new Vec(4.5, 0.5, 0.5),
                waterProfile(false), rectangular, 12);
        assertTrue(throughGap.found(),
                () -> "a shallow rectangular swimmer should fit its one-cell Z volume: "
                        + throughGap);
    }

    @Test
    void dolphinCanBreachButOrdinaryFishCannot() {
        TestWorld world = waterBox(-1, 1, 0, 0, -1, 1);
        Vec start = new Vec(0.5, 0.1, 0.5);
        Vec target = new Vec(0.5, 1.5, 0.5);

        PathResult dolphin = find(world, start, target, waterProfile(true), SMALL, 8);
        PathResult cod = find(world, start, target, waterProfile(false), SMALL, 8);

        assertTrue(dolphin.found(), dolphin.status() + " " + dolphin.nodes());
        assertEquals(1, dolphin.nodes().getLast().blockY());
        assertEquals(PathNode.Movement.BREACH,
                dolphin.nodes().getLast().movement());
        assertFalse(cod.found());
    }

    @Test
    void flyerDetoursAroundSolidThreeDimensionalObstacle() {
        TestWorld world = new TestWorld();
        for (int y = -1; y <= 2; y++) {
            for (int z = -1; z <= 1; z++) world.set(2, y, z, Block.STONE);
        }

        PathResult result = find(world, new Vec(0.5, 0.5, 0.5),
                new Vec(4.5, 0.5, 0.5), flyingProfile(), SMALL, 30);

        assertTrue(result.found());
        assertEquals(PathNode.Movement.FLY, result.nodes().getLast().movement());
        assertTrue(result.nodes().stream().anyMatch(node ->
                node.blockY() > 2 || node.blockY() < -1 || Math.abs(node.blockZ()) > 1));
        assertFalse(result.nodes().stream().anyMatch(node ->
                node.blockX() == 2 && node.blockY() >= -1 && node.blockY() <= 2
                        && Math.abs(node.blockZ()) <= 1));
    }

    @Test
    void largerFlyerDetoursInsteadOfUsingGapThatSmallFlyerCanUse() {
        TestWorld world = new TestWorld();
        for (int y = -1; y <= 2; y++) {
            world.set(2, y, -1, Block.STONE);
            world.set(2, y, 1, Block.STONE);
        }
        BoundingBox wide = new BoundingBox(1.8, 1.8, 1.8);

        PathResult small = find(world, new Vec(0.5, 0.5, 0.5),
                new Vec(4.5, 0.5, 0.5), flyingProfile(), SMALL, 8);
        PathResult large = find(world, new Vec(0.5, 0.5, 0.5),
                new Vec(4.5, 0.5, 0.5), flyingProfile(), wide, 8);

        assertTrue(small.found());
        assertTrue(large.found(), large::toString);
        double expectedSmallLength = 3 + Math.sqrt(2);
        assertEquals(expectedSmallLength, pathLength(small), 1.0e-9,
                "the start uses floor(y + 0.5), while the target uses floor(y)");
        assertTrue(pathLength(large) > expectedSmallLength,
                "wide flyer incorrectly used the one-block gap");
        for (PathNode node : large.nodes()) {
            for (int x = node.graphX(); x < node.graphX() + 2; x++) {
                for (int y = node.graphY(); y < node.graphY() + 2; y++) {
                    for (int z = node.graphZ(); z < node.graphZ() + 2; z++) {
                        assertFalse(x == 2 && y >= -1 && y <= 2
                                        && (z == -1 || z == 1),
                                () -> "wide occupied volume crossed a blocker at "
                                        + node + ": " + large);
                    }
                }
            }
        }
        assertTrue(large.nodes().stream().allMatch(node ->
                        Math.abs(node.x() - (node.graphX() + 1.0)) < 1.0e-9
                                && Math.abs(node.z() - (node.graphZ() + 1.0))
                                < 1.0e-9),
                "floor(width + 1) projection must offset wide fly nodes");
        System.out.printf(Locale.ROOT,
                "FLY-WIDTH smallLength=%.9f small=%s largeLength=%.9f large=%s%n",
                pathLength(small), small.nodes(), pathLength(large), large.nodes());
    }

    @Test
    void flyerCannotCutThreeAxisCornerPastPairwiseBlocker() {
        TestWorld world = new TestWorld().set(1, 1, 0, Block.STONE);

        PathResult result = find(world, new Vec(0.5, 0.5, 0.5),
                new Vec(1.5, 1.5, 1.5), flyingProfile(), SMALL, 12);

        assertTrue(result.found(), result::toString);
        assertTrue(result.nodes().size() > 2,
                "direct 3-axis edge requires all six baseline intermediates");
        assertTrue(pathLength(result) > Math.sqrt(3));
    }

    @Test
    void flyerCannotCutDiagonalThroughHardDynamicInfluence() {
        NavigationInfluence blockedIntermediate = (blocks, point, box) ->
                point.blockX() == 1 && point.blockY() == 1
                        && point.blockZ() == 0
                        ? InfluenceResult.forbidden("dynamic barrier")
                        : InfluenceResult.NONE;

        PathResult result = pathfinder.findPath(NavigationRequest.builder(
                        new TestWorld(), new Vec(0.5, 0.5, 0.5),
                        new Vec(1.5, 1.5, 1.5), SMALL, flyingProfile())
                .maxPathLength(12)
                .influences(List.of(blockedIntermediate))
                .build(), SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        assertTrue(result.nodes().size() > 2,
                "a hard modular influence on a required intermediate must "
                        + "prevent the direct diagonal");
    }

    @Test
    void smallFlyerUsesFirstAcceptedSnapshottedStartCandidate() {
        TestWorld world = new TestWorld().set(0, 1, 0, Block.STONE);
        EntityTraversalState state = new EntityTraversalState(
                false, false, Set.of(), -64, 319,
                List.of(new Vec(0, 1, 0), new Vec(-1, 1, 0),
                        new Vec(1, 1, 0)));
        NavigationProfile bee =
                BuiltinNavigationProfiles.forEntityType(EntityType.BEE);

        PathResult result = pathfinder.findPath(NavigationRequest.builder(
                        world, new Vec(0.5, 0.5, 0.5),
                        new Vec(3.5, 0.5, 0.5), SMALL, bee)
                .maxPathLength(12)
                .entityState(state)
                .build(), SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        assertEquals(-1, result.nodes().getFirst().graphX(),
                "candidate order must be preserved, including invalid entries");
        assertEquals(1, result.nodes().getFirst().graphY());
        assertEquals(0, result.nodes().getFirst().graphZ());
    }

    @Test
    void reachedSearchReturnsClosestDiscoveredTargetNode() {
        Vec target = new Vec(1.5, 0.5, 1.5);
        NavigationInfluence expensiveTarget = (blocks, point, box) ->
                point.blockX() == 1 && point.blockY() == 0
                        && point.blockZ() == 1
                        ? InfluenceResult.penalty(100, "target")
                        : InfluenceResult.NONE;

        PathResult result = pathfinder.findPath(NavigationRequest.builder(
                        new TestWorld(), new Vec(0.5, 0.5, 0.5), target,
                        SMALL, flyingProfile())
                .maxPathLength(12)
                .reachRange(1)
                .influences(List.of(expensiveTarget))
                .build(), SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        PathNode end = result.nodes().getLast();
        assertEquals(1, end.graphX());
        assertEquals(0, end.graphY());
        assertEquals(1, end.graphZ(),
                "reach-range termination reconstructs the closest discovered "
                        + "target node, not merely the popped in-range node");
    }

    @Test
    void nodeBudgetPartialUsesClosestDiscoveredNeighbor() {
        double oneExpansionBudget = 2.0 / (16 * 16);
        PathResult result = pathfinder.findPath(NavigationRequest.builder(
                        new TestWorld(), new Vec(0.5, 0.5, 0.5),
                        new Vec(5.5, 0.5, 0.5), SMALL, flyingProfile())
                .maxPathLength(12)
                .maxVisitedMultiplier(oneExpansionBudget)
                .build(), SearchControl.NONE);

        assertEquals(PathStatus.PARTIAL, result.status());
        assertEquals(1, result.visitedNodes(),
                "the adjusted budget of two stops before a second pop");
        assertEquals(26, result.examinedNeighbors(),
                "one open flying expansion evaluates exactly the 26 unique "
                        + "neighbor coordinates from the reference order");
        PathNode end = result.nodes().getLast();
        assertEquals(1, end.graphX());
        assertEquals(0, end.graphY());
        assertEquals(0, end.graphZ(),
                "partial endpoints update when a node is discovered");
    }

    @Test
    void nodeBudgetIsIndependentFromGeometricPathLimit() {
        NavigationRequest request = NavigationRequest.builder(
                        new TestWorld(), new Vec(0.5, 0.5, 0.5),
                        new Vec(80.5, 0.5, 0.5), SMALL, flyingProfile())
                .maxPathLength(100)
                .nodeSearchRange(1)
                .build();

        PathResult result = pathfinder.findPath(request, SearchControl.NONE);

        assertEquals(PathStatus.PARTIAL, result.status());
        assertEquals(15, result.visitedNodes(),
                "floor(nodeSearchRange * 16) controls expansions separately "
                        + "from maxPathLength");
    }

    @Test
    void repeatedFlyingWalkableEvaluationAccumulatesNodeMalus() {
        TestWorld world = new TestWorld()
                .floor(-2, 2, -2, 2, 0, Block.STONE);
        NavigationRequest request = NavigationRequest.builder(
                        world, new Vec(0.5, 1, 0.5), new Vec(2.5, 1, 0.5),
                        SMALL, flyingProfile())
                .maxPathLength(12)
                .build();
        FlyNodeEvaluator evaluator = new FlyNodeEvaluator(request);
        Vec floorAdjacent = new Vec(0, 1, 0);

        assertEquals(1, evaluator.malusAt(floorAdjacent));
        assertEquals(2, evaluator.malusAt(floorAdjacent));
        assertEquals(3, evaluator.malusAt(floorAdjacent),
                "the cached flying node receives another WALKABLE increment "
                        + "on every evaluator request");
    }

    @Test
    void repeatedSwimmingBreachEvaluationAccumulatesAnchorPenalty() {
        TestWorld world = new TestWorld().set(0, 0, 0, Block.WATER);
        MobTraversalProfile swimmer = MobTraversalProfile.builder("breacher")
                .malus(TerrainType.BREACH, 0)
                .build();
        NavigationProfile profile = NavigationProfile.builder(NavigationMode.WATER, swimmer, GroundCapabilities.STANDARD).allowBreaching(true).prefersShallowWater(false).avoidSun(false).build();
        NavigationRequest request = NavigationRequest.builder(
                        world, new Vec(0.5, 0.5, 0.5), new Vec(0.5, 1.5, 0.5),
                        SMALL, profile)
                .maxPathLength(8)
                .build();
        SwimNodeEvaluator evaluator = new SwimNodeEvaluator(request);
        Vec breach = new Vec(0, 1, 0);

        assertEquals(8, evaluator.malusAt(breach));
        assertEquals(16, evaluator.malusAt(breach),
                "an empty-fluid BREACH anchor receives +8 on every request");
    }

    @Test
    void wideSwimmerBreachPenaltyDependsOnAnchorFluidOnly() {
        TestWorld world = waterBox(0, 1, 0, 0, 0, 1)
                .set(1, 0, 0, Block.AIR)
                .set(1, -1, 0, Block.WATER);
        MobTraversalProfile swimmer = MobTraversalProfile.builder("wide_breacher")
                .malus(TerrainType.BREACH, 0)
                .build();
        NavigationProfile profile = NavigationProfile.builder(NavigationMode.WATER, swimmer, GroundCapabilities.STANDARD).allowBreaching(true).prefersShallowWater(false).avoidSun(false).build();
        BoundingBox wide = new BoundingBox(1.01, 0.5, 1.01);
        NavigationRequest request = NavigationRequest.builder(
                        world, new Vec(0.5, 0.5, 0.5), new Vec(1.5, 1.5, 0.5),
                        wide, profile)
                .maxPathLength(8)
                .build();
        SwimNodeEvaluator evaluator = new SwimNodeEvaluator(request);

        assertEquals(0, evaluator.malusAt(new Vec(0, 0, 0)),
                "a later dry cell makes the volume BREACH, but the water "
                        + "anchor must not receive the empty-fluid penalty");
    }

    private PathResult find(TestWorld world, Vec start, Vec target,
                            NavigationProfile profile, BoundingBox box,
                            double maxLength) {
        return pathfinder.findPath(NavigationRequest.builder(
                        world, start, target, box, profile)
                .maxPathLength(maxLength)
                .build(), SearchControl.NONE);
    }

    private static NavigationProfile waterProfile(boolean breach) {
        return NavigationProfile.builder(NavigationMode.WATER, MobTraversalProfile.WATER_ANIMAL, GroundCapabilities.STANDARD).allowBreaching(breach).prefersShallowWater(false).avoidSun(false).build();
    }

    private static NavigationProfile flyingProfile() {
        return NavigationProfile.builder(NavigationMode.FLYING, MobTraversalProfile.DEFAULT, GroundCapabilities.STANDARD).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();
    }

    private static TestWorld waterBox(int minX, int maxX, int minY, int maxY,
                                      int minZ, int maxZ) {
        TestWorld world = new TestWorld();
        for (int x = minX; x <= maxX; x++) {
            for (int y = minY; y <= maxY; y++) {
                for (int z = minZ; z <= maxZ; z++) world.set(x, y, z, Block.WATER);
            }
        }
        return world;
    }

    private static boolean hasMultiAxisStep(PathResult result) {
        for (int i = 1; i < result.nodes().size(); i++) {
            PathNode before = result.nodes().get(i - 1);
            PathNode after = result.nodes().get(i);
            int changed = (before.blockX() == after.blockX() ? 0 : 1)
                    + (before.blockY() == after.blockY() ? 0 : 1)
                    + (before.blockZ() == after.blockZ() ? 0 : 1);
            if (changed >= 2) return true;
        }
        return false;
    }

    private static double pathLength(PathResult result) {
        double length = 0;
        for (int i = 1; i < result.nodes().size(); i++) {
            length += result.nodes().get(i - 1).asVec()
                    .distance(result.nodes().get(i).asVec());
        }
        return length;
    }
}
