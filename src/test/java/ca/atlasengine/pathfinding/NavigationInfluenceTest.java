package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.CancellationToken;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.influence.BlockAvoidanceInfluence;
import ca.atlasengine.pathfinding.influence.InfluenceResult;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.influence.NavigationZoneInfluence;
import ca.atlasengine.pathfinding.influence.ReturnRadiusInfluence;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

class NavigationInfluenceTest {
    private static final BoundingBox BOX = new BoundingBox(0.6, 1.8, 0.6);

    @Test
    void blockAvoidanceRadiusAndCancellationBoundScanWork() {
        assertThrows(IllegalArgumentException.class,
                () -> new BlockAvoidanceInfluence(
                        Set.of(Block.LAVA),
                        BlockAvoidanceInfluence.MAX_RADIUS + 1,
                        true, 0));

        TestWorld world = flat();
        BlockAvoidanceInfluence influence = new BlockAvoidanceInfluence(
                Set.of(Block.LAVA), BlockAvoidanceInfluence.MAX_RADIUS,
                true, 0);
        long before = world.reads();
        InfluenceResult interrupted = influence.evaluate(
                world, new Vec(0.5, 1, 0.5), BOX,
                new SearchControl(CancellationToken.NONE,
                        System.nanoTime() - 1));

        assertTrue(interrupted.blocked());
        assertEquals(0, world.reads() - before,
                "an expired search must stop before the first cubic scan read");
    }

    @Test
    void entityDispatcherComposesDynamicInfluences() {
        TestWorld world = flat();
        NavigationZoneInfluence zone = new NavigationZoneInfluence(
                new Vec(3, 0, -1), new Vec(7, 4, 1), true, 0, "temporary-danger");
        NavigationRequest request = NavigationRequest.builder(
                        world, new Pos(0.5, 1, 0.5), new Pos(10.5, 1, 0.5), BOX,
                        BuiltinNavigationProfiles.forEntityType(
                                net.minestom.server.entity.EntityType.ZOMBIE))
                .maxPathLength(30)
                .maxVisitedMultiplier(8)
                .influences(List.of(zone))
                .build();

        PathResult result = new EntityPathfinder().findPath(request, SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        assertTrue(result.nodes().stream().noneMatch(node ->
                node.x() >= 3 && node.x() <= 7 && node.z() >= -1 && node.z() <= 1),
                result::toString);
    }

    @Test
    void returnRadiusBlocksOutwardTravelButAllowsReturnFromOutside() {
        TestWorld world = new TestWorld()
                .floor(-50, 50, -4, 4, 0, Block.STONE);
        Vec home = new Vec(0, 1, 0);
        Vec outside = new Vec(40.5, 1, 0.5);
        NavigationInfluence boundary =
                new ReturnRadiusInfluence(home, outside, 32);
        NavigationProfile zombie = BuiltinNavigationProfiles.forEntityType(
                net.minestom.server.entity.EntityType.ZOMBIE);

        PathResult outward = new EntityPathfinder().findPath(
                NavigationRequest.builder(
                                world, outside, new Vec(45.5, 1, 0.5), BOX,
                                zombie)
                        .maxPathLength(20)
                        .maxVisitedMultiplier(8)
                        .influences(List.of(boundary))
                        .build(),
                SearchControl.NONE);
        PathResult returning = new EntityPathfinder().findPath(
                NavigationRequest.builder(
                                world, outside, new Vec(25.5, 1, 0.5), BOX,
                                zombie)
                        .maxPathLength(30)
                        .maxVisitedMultiplier(8)
                        .influences(List.of(boundary))
                        .build(),
                SearchControl.NONE);

        assertFalse(outward.found(), outward::toString);
        assertTrue(returning.found(), returning::toString);
        assertEquals(25, returning.nodes().getLast().graphX());
    }

    private static TestWorld flat() {
        return new TestWorld().floor(-15, 15, -15, 15, 0, Block.STONE);
    }
}
