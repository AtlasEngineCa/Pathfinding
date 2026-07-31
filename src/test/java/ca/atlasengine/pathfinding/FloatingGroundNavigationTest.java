package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.EntityTraversalState;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Focused tests for floating ground-navigation branches.
 */
class FloatingGroundNavigationTest {
    private static final BoundingBox MOB = new BoundingBox(0.6, 1.8, 0.6);
    private final EntityPathfinder pathfinder = new EntityPathfinder();

    @Test
    void profilesWithFloatingBehaviorRetainCapability() {
        // The broad animal family does not float. Only concrete
        // mobs which install FloatGoal do so.
        assertTrue(!MobTraversalProfile.ANIMAL.canFloat());
        assertTrue(MobTraversalProfile.FLOATING_ANIMAL.canFloat());
        assertTrue(MobTraversalProfile.VILLAGER.canFloat());
        assertTrue(MobTraversalProfile.ENDERMAN.canFloat());
        assertTrue(MobTraversalProfile.FOX.canFloat());
        assertTrue(MobTraversalProfile.BEE.canFloat());
        assertTrue(MobTraversalProfile.PARROT.canFloat());
        assertTrue(MobTraversalProfile.SNIFFER.canFloat());
        assertTrue(MobTraversalProfile.GOAT.canFloat());
        assertTrue(MobTraversalProfile.WOLF.canFloat());
        assertTrue(MobTraversalProfile.CAMEL.canFloat());
        assertTrue(MobTraversalProfile.CHICKEN.canFloat());
        assertTrue(MobTraversalProfile.RAVAGER.canFloat());
        assertTrue(MobTraversalProfile.WARDEN.canFloat());
        assertTrue(MobTraversalProfile.CREAKING.canFloat());
    }

    @Test
    void floatingStartRisesToTopOfContiguousWaterColumn() {
        TestWorld world = pool(0, 4, 0, 2);
        MobTraversalProfile floating = MobTraversalProfile.builder("floating")
                .malus(TerrainType.WATER, 0)
                .canFloat(true)
                .build();

        PathResult result = find(world, floating,
                new Pos(0.5, 1.2, 0.5), new Pos(4.5, 3, 0.5));

        assertTrue(result.found());
        assertEquals(3, result.nodes().getFirst().graphY(),
                "baseline starts on the highest contiguous WATER block");
        assertTrue(result.nodes().stream().allMatch(node -> node.graphY() == 3),
                "canFloat adds horizontal surface nodes, not 3-D swim nodes");
    }

    @Test
    void floatingStartIncludesWaterloggedSourceCellAboveWater() {
        TestWorld world = new TestWorld()
                .floor(-1, 3, -1, 1, -1, Block.STONE)
                .set(0, 0, 0, Block.WATER)
                .set(0, 1, 0, Block.OAK_SLAB
                        .withProperty("type", "bottom")
                        .withProperty("waterlogged", "true"));
        MobTraversalProfile floating = MobTraversalProfile.builder("floating")
                .malus(TerrainType.WATER, 0)
                .canFloat(true)
                .build();

        NavigationProfile profile = NavigationProfile.builder(NavigationMode.GROUND, floating, GroundCapabilities.STANDARD).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();
        PathResult result = pathfinder.findPath(
                NavigationRequest.builder(
                                world, new Pos(0.5, 0.2, 0.5),
                                new Pos(2.5, 1, 0.5), MOB, profile)
                        .maxPathLength(12)
                        .entityState(new EntityTraversalState(
                                false, true, Set.of(), -64))
                        .build(),
                SearchControl.NONE);

        assertTrue(!result.nodes().isEmpty(), result::toString);
        assertEquals(1, result.nodes().getFirst().graphY(),
                "the start scan treats a waterlogged source-fluid state as "
                        + "part of the contiguous water column");
    }

    @Test
    void nonFloatingWalkerCollapsesWaterCandidateToPoolBottom() {
        TestWorld world = pool(0, 4, 0, 2);
        MobTraversalProfile sinking = MobTraversalProfile.builder("sinking")
                .malus(TerrainType.WATER, 0)
                .build();

        PathResult result = find(world, sinking,
                new Pos(0.5, 3, 0.5), new Pos(4.5, 1, 0.5));

        assertTrue(result.found(), result::toString);
        assertTrue(result.nodes().stream().allMatch(node -> node.graphY() == 1),
                "without floating support ground navigation descends through WATER");
    }

    private PathResult find(TestWorld world, MobTraversalProfile profile,
                            Pos start, Pos goal) {
        NavigationProfile navigation = NavigationProfile.builder(NavigationMode.GROUND, profile, GroundCapabilities.STANDARD).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();
        return pathfinder.findPath(NavigationRequest.builder(
                                world, start, goal, MOB, navigation)
                        .maxPathLength(32)
                        .maxVisitedMultiplier(8)
                        .entityState(new EntityTraversalState(
                                false, true, Set.of(), -64))
                        .build(),
                SearchControl.NONE);
    }

    private static TestWorld pool(int minX, int maxX, int minZ, int depth) {
        TestWorld world = new TestWorld()
                .floor(minX - 1, maxX + 1, minZ - 1, minZ + 1,
                        0, Block.STONE);
        for (int x = minX; x <= maxX; x++) {
            for (int y = 1; y <= depth + 1; y++) {
                world.set(x, y, minZ, Block.WATER);
            }
        }
        return world;
    }
}
