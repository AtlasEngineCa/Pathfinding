package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.EntityTraversalState;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.influence.BlockAvoidanceInfluence;
import ca.atlasengine.pathfinding.influence.EntityFearInfluence;
import ca.atlasengine.pathfinding.influence.EntitySnapshot;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.influence.NavigationZoneInfluence;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Set;
import java.util.UUID;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Dynamic influences observed through the route they actually produce, not
 * through their own {@code evaluate} return value. An influence attached to a
 * {@link NavigationRequest} shapes the graph the search expands and must never
 * disturb anything the search decided before its first expansion.
 */
class LiveInfluenceRoutingTest {
    private static final BoundingBox BOX = new BoundingBox(0.6, 1.8, 0.6);
    private static final NavigationProfile ZOMBIE =
            BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
    private final EntityPathfinder pathfinder = new EntityPathfinder();

    /**
     * A feared entity beside the straight lane bends the route wide around it
     * and keeps a hard minimum standoff no waypoint may cross.
     */
    @Test
    void fearedEntityCreatesWideDetourWithMinimumStandoff() {
        TestWorld world = flat();
        EntitySnapshot threat = new EntitySnapshot(
                new UUID(7, 9), EntityType.WOLF, new Vec(5.5, 1, 0.5));

        PathResult afraid = route(world, List.of(
                new EntityFearInfluence(List.of(threat), 5, 2, 20)));
        PathResult fearless = route(flat(), List.of());

        assertTrue(fearless.found(), fearless::toString);
        assertTrue(fearless.nodes().stream().allMatch(node ->
                        node.graphZ() == 0),
                () -> "the unafraid baseline is not the straight lane: "
                        + fearless.nodes());

        assertTrue(afraid.found(), afraid::toString);
        assertTrue(afraid.nodes().stream().anyMatch(node ->
                        Math.abs(node.z()) >= 3.5),
                () -> "fear produced no wide detour: " + afraid.nodes());
        assertTrue(afraid.nodes().stream().noneMatch(node ->
                        node.asVec().distance(threat.position()) <= 2),
                () -> "a waypoint crossed the forbidden radius: "
                        + afraid.nodes());
        assertEquals(10, afraid.nodes().getLast().graphX(),
                "the detour still reaches the target");
    }

    /**
     * A forbidding block avoidance clears a full radius-one cube around every
     * match, and does so without editing the mob profile: the same profile
     * without the influence walks straight over the flower.
     */
    @Test
    void avoidedBlockIsForbiddenInAFullThreeByThreeWithoutChangingTheProfile() {
        TestWorld avoided = flat().set(5, 1, 0, Block.DANDELION);
        TestWorld plain = flat().set(5, 1, 0, Block.DANDELION);

        PathResult detour = route(avoided, List.of(
                new BlockAvoidanceInfluence(Set.of(Block.DANDELION), 1, true, 0)));
        PathResult straight = route(plain, List.of());

        assertTrue(detour.found(), detour::toString);
        assertTrue(detour.nodes().stream().noneMatch(node ->
                        Math.abs(node.graphX() - 5) <= 1
                                && Math.abs(node.graphZ()) <= 1),
                () -> "route entered the forbidden 3x3: " + detour.nodes());
        assertEquals(10, detour.nodes().getLast().graphX());

        assertTrue(straight.found(), straight::toString);
        assertTrue(straight.nodes().stream().anyMatch(node ->
                        node.graphX() == 5 && node.graphZ() == 0),
                () -> "the influence, not the profile, must forbid the "
                        + "flower: " + straight.nodes());
        assertEquals(0, MobTraversalProfile.DEFAULT.malus(TerrainType.WALKABLE));
    }

    /**
     * The start node is resolved before the first expansion, so attaching an
     * influence, even one that forbids a region nowhere near the search,
     * must not move a floating mob off the surface of its water column.
     */
    @Test
    void influencesDoNotDisturbWaterSurfaceStartNormalization() {
        NavigationInfluence faraway = new NavigationZoneInfluence(
                new Vec(40, 0, 40), new Vec(42, 2, 42), true, 0, "far_away");

        PathResult plain = swim(pool(), List.of());
        PathResult influenced = swim(pool(), List.of(faraway));

        assertTrue(plain.found(), plain::toString);
        assertEquals(3, plain.nodes().getFirst().graphY(),
                "a floating start rises to the highest contiguous water cell");
        assertTrue(plain.nodes().stream().allMatch(node -> node.graphY() == 3),
                () -> "floating adds horizontal surface nodes, not 3-D swim "
                        + "nodes: " + plain.nodes());
        assertTrue(influenced.found(), influenced::toString);
        assertEquals(3, influenced.nodes().getFirst().graphY(),
                "attaching influences must keep the water-surface start");
        assertEquals(plain.nodes(), influenced.nodes(),
                "an influence that matches nothing must not change the route");
    }

    private PathResult route(TestWorld world,
                             List<NavigationInfluence> influences) {
        return pathfinder.findPath(NavigationRequest.builder(
                                world, new Pos(0.5, 1, 0.5),
                                new Pos(10.5, 1, 0.5), BOX, ZOMBIE)
                        .maxPathLength(30)
                        .maxVisitedMultiplier(8)
                        .influences(influences)
                        .build(),
                SearchControl.NONE);
    }

    private PathResult swim(TestWorld world,
                            List<NavigationInfluence> influences) {
        MobTraversalProfile floating = MobTraversalProfile.builder("floating")
                .malus(TerrainType.WATER, 0)
                .canFloat(true)
                .build();
        NavigationProfile profile = NavigationProfile.builder(NavigationMode.GROUND, floating, GroundCapabilities.STANDARD).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();
        return pathfinder.findPath(NavigationRequest.builder(
                                world, new Pos(0.5, 1.2, 0.5),
                                new Pos(4.5, 3, 0.5), BOX, profile)
                        .maxPathLength(32)
                        .maxVisitedMultiplier(8)
                        .entityState(new EntityTraversalState(
                                false, true, Set.of(), -64))
                        .influences(influences)
                        .build(),
                SearchControl.NONE);
    }

    private static TestWorld flat() {
        return new TestWorld().floor(-15, 15, -15, 15, 0, Block.STONE);
    }

    private static TestWorld pool() {
        TestWorld world = new TestWorld().floor(-1, 5, -1, 1, 0, Block.STONE);
        for (int x = 0; x <= 4; x++) {
            for (int y = 1; y <= 3; y++) world.set(x, y, 0, Block.WATER);
        }
        return world;
    }
}
