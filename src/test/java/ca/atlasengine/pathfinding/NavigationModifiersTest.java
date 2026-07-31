package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.NavigationModifiers;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class NavigationModifiersTest {
    @Test
    void climbablePlanningIsRequestScopedAndLeavesBuiltinDefaultDisabled() {
        NavigationProfile base = BuiltinNavigationProfiles.forEntityType(
                EntityType.ZOMBIE);
        NavigationModifiers modifier = NavigationModifiers.builder()
                .climbables(ClimbableCapabilities.STANDARD)
                .build();

        NavigationProfile climbing = modifier.applyTo(base);

        assertFalse(base.groundCapabilities().climbables().enabled());
        assertTrue(climbing.groundCapabilities().climbables().enabled());
        assertSame(base, NavigationModifiers.NONE.applyTo(base));
    }

    @Test
    void buriedTargetSwitchIsRequestScopedAndKeepsContainerTargetUnderground() {
        TestWorld world = new TestWorld();
        for (int x = -1; x <= 5; x++) {
            world.set(x, 0, 0, Block.STONE);
            world.set(x, 3, 0, Block.STONE);
            world.set(x, 1, -1, Block.STONE);
            world.set(x, 1, 1, Block.STONE);
            world.set(x, 2, -1, Block.STONE);
            world.set(x, 2, 1, Block.STONE);
        }
        for (int y = 1; y <= 6; y++) world.set(4, y, 0, Block.STONE);
        NavigationProfile base = BuiltinNavigationProfiles.forEntityType(
                EntityType.COPPER_GOLEM);
        NavigationProfile buried = NavigationModifiers.builder()
                .pathToTargetsBelowSurface(true).build().applyTo(base);
        NavigationRequest ordinary = NavigationRequest.builder(
                        world, new Pos(0.5, 1, 0.5),
                        new Pos(4.5, 1, 0.5), MOB, base)
                .maxPathLength(16).nodeSearchRange(16).reachRange(1).build();
        NavigationRequest transport = NavigationRequest.builder(
                        world, new Pos(0.5, 1, 0.5),
                        new Pos(4.5, 1, 0.5), MOB, buried)
                .maxPathLength(16).nodeSearchRange(16).reachRange(1).build();

        PathResult surface = new EntityPathfinder().findPath(
                ordinary, SearchControl.NONE);
        PathResult underground = new EntityPathfinder().findPath(
                transport, SearchControl.NONE);

        assertFalse(base.pathToTargetsBelowSurface());
        assertTrue(buried.pathToTargetsBelowSurface());
        assertFalse(surface.found(),
                "ordinary ground navigation should surface a solid target");
        assertTrue(underground.found(), underground::toString);
        assertEquals(3, underground.nodes().getLast().graphX());
        assertEquals(1, underground.nodes().getLast().graphY());
    }

    private static final BoundingBox MOB = new BoundingBox(0.6, 1.8, 0.6);

    @Test
    void temporaryWaterPreferenceChangesOnlyTheModifiedRequest() {
        TestWorld world = wetLane();
        NavigationProfile base =
                BuiltinNavigationProfiles.forEntityType(EntityType.SNIFFER);
        NavigationRequest ordinary = request(world, base);
        NavigationRequest following = ordinary.withModifiers(
                NavigationModifiers.builder()
                        .terrainCost(TerrainType.WATER, 0)
                        .build());

        PathResult dryRoute = new EntityPathfinder()
                .findPath(ordinary, SearchControl.NONE);
        PathResult wetRoute = new EntityPathfinder()
                .findPath(following, SearchControl.NONE);

        assertEquals(-1, base.mobProfile().malus(TerrainType.WATER));
        assertEquals(-1, ordinary.profile().mobProfile().malus(TerrainType.WATER));
        assertEquals(0, following.profile().mobProfile().malus(TerrainType.WATER));
        assertTrue(dryRoute.nodes().stream().anyMatch(node -> node.graphZ() != 0),
                dryRoute::toString);
        assertTrue(wetRoute.nodes().stream().allMatch(node -> node.graphZ() == 0),
                wetRoute::toString);
    }

    @Test
    void temporaryDoorCapabilityOpensAPreviouslyBlockedRoute() {
        TestWorld world = doorCorridor();
        NavigationRequest ordinary = request(
                world, BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE));
        NavigationRequest doorBreaking = ordinary.withModifiers(
                NavigationModifiers.builder()
                        .canOpenDoors(true)
                        .canPassDoors(true)
                        .build());

        PathResult blocked = new EntityPathfinder()
                .findPath(ordinary, SearchControl.NONE);
        PathResult allowed = new EntityPathfinder()
                .findPath(doorBreaking, SearchControl.NONE);

        assertFalse(blocked.found());
        assertTrue(allowed.found(), allowed::toString);
        assertEquals(List.of(0, 1, 2, 3, 4, 5),
                allowed.nodes().stream().map(PathNode::graphX).toList());
        assertFalse(ordinary.profile().mobProfile().canOpenDoors());
    }

    @Test
    void concurrentModifiedAndOrdinaryRequestsRemainIsolated() {
        TestWorld world = wetLane();
        NavigationRequest ordinary = request(
                world, BuiltinNavigationProfiles.forEntityType(EntityType.SNIFFER));
        NavigationRequest following = ordinary.withModifiers(
                NavigationModifiers.builder()
                        .terrainCost(TerrainType.WATER, 0)
                        .build());

        try (var service = new AsyncEntityPathfindingService(2, 8)) {
            var dryFuture = service.submit(ordinary);
            var wetFuture = service.submit(following);
            PathResult dryRoute = dryFuture.join();
            PathResult wetRoute = wetFuture.join();

            assertTrue(dryRoute.nodes().stream().anyMatch(
                    node -> node.graphZ() != 0));
            assertTrue(wetRoute.nodes().stream().allMatch(
                    node -> node.graphZ() == 0));
            assertEquals(-1, ordinary.profile().mobProfile()
                    .malus(TerrainType.WATER));
        }
    }

    @Test
    void modifierCopiesAndValidatesInput() {
        PlatformJumpCapabilities jump =
                PlatformJumpCapabilities.builder().maxHorizontalDistance(3).maxRise(1).maxDrop(2).apexClearance(1).build();
        NavigationModifiers modifier = NavigationModifiers.builder()
                .terrainCost(TerrainType.LAVA, 3)
                .canFloat(true)
                .avoidSun(true)
                .platformJump(jump)
                .build();
        NavigationProfile base =
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
        NavigationProfile modified = modifier.applyTo(base);

        assertEquals(3, modified.mobProfile().malus(TerrainType.LAVA));
        assertTrue(modified.mobProfile().canFloat());
        assertTrue(modified.avoidSun());
        assertEquals(jump, modified.groundCapabilities().platformJump());
        assertEquals(PlatformJumpCapabilities.DISABLED,
                base.groundCapabilities().platformJump());
        assertThrows(UnsupportedOperationException.class,
                () -> modifier.terrainCosts().put(TerrainType.WATER, 4.0));
        assertThrows(IllegalArgumentException.class,
                () -> NavigationModifiers.builder()
                        .terrainCost(TerrainType.WATER, Double.NaN));
        assertThrows(IllegalArgumentException.class,
                () -> NavigationModifiers.builder().platformJump(null));
    }

    private static NavigationRequest request(
            TestWorld world, NavigationProfile profile) {
        return NavigationRequest.builder(
                        world, new Pos(0.5, 1, 0.5), new Pos(5.5, 1, 0.5),
                        MOB, profile)
                .maxPathLength(24)
                .maxVisitedMultiplier(8)
                .build();
    }

    private static TestWorld wetLane() {
        TestWorld world = new TestWorld()
                .floor(-20, 20, -5, 5, 0, Block.STONE);
        for (int x = 1; x <= 4; x++) world.set(x, 1, 0, Block.WATER);
        return world;
    }

    private static TestWorld doorCorridor() {
        TestWorld world = new TestWorld()
                .floor(-20, 20, -1, 1, 0, Block.STONE);
        for (int x = -20; x <= 20; x++) {
            world.column(x, -1, 1, 4, Block.STONE);
            world.column(x, 1, 1, 4, Block.STONE);
        }
        Block door = Block.OAK_DOOR.withProperty("facing", "east")
                .withProperty("hinge", "left").withProperty("open", "false");
        world.set(2, 1, 0, door.withProperty("half", "lower"));
        world.set(2, 2, 0, door.withProperty("half", "upper"));
        return world;
    }
}
