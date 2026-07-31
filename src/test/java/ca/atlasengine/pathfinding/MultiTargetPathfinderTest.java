package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class MultiTargetPathfinderTest {
    private static final BoundingBox ZOMBIE_BOX =
            new BoundingBox(0.6, 1.8, 0.6);
    private static final BoundingBox SMALL_FLYER =
            new BoundingBox(0.6, 0.6, 0.6);

    @Test
    void groundSearchUsesOneFrontierAndReachesNearerDestination() {
        TestWorld world = new TestWorld().floor(
                -3, 12, -3, 3, 0, Block.STONE);
        NavigationProfile zombie =
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
        NavigationRequest request = NavigationRequest.builder(
                        world, new Pos(0.5, 1, 0.5),
                        new Pos(99.5, 1, 0.5), ZOMBIE_BOX, zombie)
                .maxPathLength(20)
                .build();

        PathResult result = new EntityPathfinder().findPathToAny(
                request, List.of(
                        new Pos(8.5, 1, 0.5),
                        new Pos(3.5, 1, 0.5)),
                SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        assertEquals(3, result.nodes().getLast().graphX());
        assertEquals(4, result.nodes().size());
    }

    @Test
    void flyingSearchUsesOneFrontierAndReachesNearerDestination() {
        NavigationRequest request = NavigationRequest.builder(
                        new TestWorld(), new Vec(0.5, 0.5, 0.5),
                        new Vec(99.5, 0.5, 0.5), SMALL_FLYER,
                        BuiltinNavigationProfiles.forEntityType(EntityType.BEE))
                .maxPathLength(20)
                .build();

        PathResult result = new EntityPathfinder().findPathToAny(
                request, List.of(
                        new Vec(8.5, 0.5, 0.5),
                        new Vec(2.5, 0.5, 0.5)),
                SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        assertEquals(2, result.nodes().getLast().graphX());
    }

    @Test
    void swimmingSearchUsesOneFrontierAndReachesNearerDestination() {
        TestWorld world = new TestWorld();
        for (int x = -2; x <= 10; x++) {
            for (int y = -1; y <= 3; y++) {
                for (int z = -2; z <= 2; z++) {
                    world.set(x, y, z, Block.WATER);
                }
            }
        }
        NavigationRequest request = NavigationRequest.builder(
                        world, new Vec(0.5, 0.5, 0.5),
                        new Vec(99.5, 0.5, 0.5),
                        new BoundingBox(0.5, 0.3, 0.5),
                        BuiltinNavigationProfiles.forEntityType(EntityType.COD))
                .maxPathLength(20)
                .build();

        PathResult result = new EntityPathfinder().findPathToAny(
                request, List.of(
                        new Vec(7.5, 0.5, 0.5),
                        new Vec(2.5, 0.5, 0.5)),
                SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        assertEquals(2, result.nodes().getLast().graphX());
    }

    @Test
    void amphibiousSearchUsesOneFrontierAndReachesNearerDestination() {
        TestWorld world = new TestWorld().floor(
                -3, 10, -3, 3, 0, Block.STONE);
        NavigationRequest request = NavigationRequest.builder(
                        world, new Pos(0.5, 1, 0.5),
                        new Pos(99.5, 1, 0.5),
                        new BoundingBox(0.5, 0.5, 0.5),
                        BuiltinNavigationProfiles.forEntityType(
                                EntityType.FROG))
                .maxPathLength(20)
                .build();

        PathResult result = new EntityPathfinder().findPathToAny(
                request, List.of(
                        new Pos(7.5, 1, 0.5),
                        new Pos(2.5, 1, 0.5)),
                SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        assertEquals(2, result.nodes().getLast().graphX());
    }

    @Test
    void unreachedTargetTieUsesShorterPartialPath() {
        TestWorld world = new TestWorld().floor(
                -5, 8, -7, 5, 0, Block.STONE);
        for (int z = -4; z <= 4; z++) {
            world.column(2, z, 1, 3, Block.STONE);
        }
        NavigationProfile zombie =
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
        NavigationRequest request = NavigationRequest.builder(
                        world, new Pos(0.5, 1, 0.5),
                        new Pos(99.5, 1, 0.5), ZOMBIE_BOX, zombie)
                .maxPathLength(2.5)
                .build();

        PathResult result = new EntityPathfinder().findPathToAny(
                request, List.of(
                        new Pos(3.5, 1, 0.5),
                        new Pos(0.5, 1, -3.5)),
                SearchControl.NONE);

        assertEquals(PathStatus.PARTIAL, result.status());
        PathNode end = result.nodes().getLast();
        assertEquals(1, end.graphX());
        assertEquals(0, end.graphZ(),
                "both target partials end two Manhattan blocks away, so the "
                        + "two-node east path wins over the longer north path");
        assertEquals(2, result.nodes().size());
    }

    @Test
    void emptyDestinationSetIsRejected() {
        NavigationRequest request = NavigationRequest.builder(
                        new TestWorld(), new Vec(0.5, 0.5, 0.5),
                        new Vec(1.5, 0.5, 0.5), SMALL_FLYER,
                        BuiltinNavigationProfiles.forEntityType(EntityType.BEE))
                .maxPathLength(8)
                .build();

        PathResult result = new EntityPathfinder().findPathToAny(
                request, List.of(), SearchControl.NONE);

        assertEquals(PathStatus.INVALID_REQUEST, result.status());
    }
}
