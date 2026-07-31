package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.EntityTraversalState;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.adaptive.SharedMeshPolicy;
import ca.atlasengine.pathfinding.internal.adaptive.SharedMeshCoordinator;
import ca.atlasengine.pathfinding.adaptive.SharedMeshHandle;
import ca.atlasengine.pathfinding.adaptive.SharedMeshSource;
import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.internal.adaptive.NavigationPlanParity;
import ca.atlasengine.pathfinding.influence.NavigationZoneInfluence;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

class AdaptiveMeshParityContractTest {
    private static final BoundingBox BOX = new BoundingBox(0.6, 1.8, 0.6);

    @Test
    void everyNavigationFamilyAndMovementKindReplaysExactCertifiedSemantics() {
        List<Fixture> fixtures = List.of(
                fixture("ground", profile(EntityType.ZOMBIE),
                        PathNode.Movement.WALK,
                        PathNode.Movement.STEP_UP,
                        PathNode.Movement.FALL),
                fixture("flying", profile(EntityType.BEE),
                        PathNode.Movement.FLY),
                fixture("water", profile(EntityType.DOLPHIN),
                        PathNode.Movement.SWIM,
                        PathNode.Movement.BREACH),
                fixture("amphibious", profile(EntityType.AXOLOTL),
                        PathNode.Movement.WALK,
                        PathNode.Movement.SWIM,
                        PathNode.Movement.FALL),
                fixture("wall-climber", profile(EntityType.SPIDER),
                        PathNode.Movement.WALK),
                fixture("climbing", climbingProfile(),
                        PathNode.Movement.WALK,
                        PathNode.Movement.CLIMB),
                fixture("jumping", jumpingProfile(),
                        PathNode.Movement.WALK,
                        PathNode.Movement.JUMP));

        for (Fixture fixture : fixtures) {
            assertCertifiedReplay(fixture);
        }
    }

    @Test
    void semanticallyEquivalentRejectsAnyMovementGraphOrPhysicalCoordinateChange() {
        Fixture fixture = fixture("climb", climbingProfile(),
                PathNode.Movement.WALK, PathNode.Movement.CLIMB);
        NavigationPlan original = fixture.plan;
        List<PathNode> changed = new ArrayList<>(original.nodes());
        PathNode node = changed.getLast();
        changed.set(changed.size() - 1, new PathNode(
                node.x(), node.y(), node.z(), PathNode.Movement.WALK,
                node.graphX(), node.graphY(), node.graphZ()));
        NavigationPlan movementChanged = copy(original, changed);
        assertFalse(NavigationPlanParity.semanticallyEquivalent(
                original, movementChanged));

        changed.set(changed.size() - 1, new PathNode(
                node.x() + 0.01, node.y(), node.z(), node.movement(),
                node.graphX(), node.graphY(), node.graphZ()));
        assertFalse(NavigationPlanParity.semanticallyEquivalent(
                original, copy(original, changed)));

        changed.set(changed.size() - 1, new PathNode(
                node.x(), node.y(), node.z(), node.movement(),
                node.graphX() + 1, node.graphY(), node.graphZ()));
        assertFalse(NavigationPlanParity.semanticallyEquivalent(
                original, copy(original, changed)));
    }

    @Test
    void realIndividualPathfinderIsTheOracleForEveryMeshFamily() {
        TestWorld ground = new TestWorld().floor(-1, 5, -1, 1, 0, Block.STONE);
        ground.set(2, 1, 0, Block.STONE);
        for (int x = -1; x <= 5; x++) {
            ground.column(x, -1, 1, 3, Block.STONE);
            ground.column(x, 1, 1, 3, Block.STONE);
        }
        assertRealReplay(request(ground, new Pos(0.5, 1, 0.5),
                        new Pos(4.5, 1, 0.5), BOX,
                        profile(EntityType.ZOMBIE), EntityTraversalState.GROUNDED),
                Set.of(PathNode.Movement.STEP_UP, PathNode.Movement.FALL));

        assertRealReplay(request(new TestWorld(),
                        new Pos(0.5, 0.5, 0.5), new Pos(4.5, 2.5, 0.5),
                        new BoundingBox(0.6, 0.6, 0.6),
                        profile(EntityType.BEE), EntityTraversalState.GROUNDED),
                Set.of(PathNode.Movement.FLY));

        TestWorld water = waterBox(0, 5, -1, 3, -1, 1);
        assertRealReplay(request(water,
                        new Pos(0.5, 0.5, 0.5), new Pos(4.5, 2.5, 0.5),
                        new BoundingBox(0.6, 0.6, 0.6),
                        profile(EntityType.COD),
                        new EntityTraversalState(
                                false, true, Set.of(), -64, 319, List.of())),
                Set.of(PathNode.Movement.SWIM));

        TestWorld amphibious = new TestWorld().floor(
                -1, 9, -2, 2, 0, Block.STONE);
        for (int x = 2; x <= 6; x++) {
            for (int z = -1; z <= 1; z++) {
                amphibious.set(x, 0, z, Block.WATER);
                amphibious.set(x, 1, z, Block.WATER);
            }
        }
        assertRealReplay(request(amphibious,
                        new Pos(0.5, 1, 0.5), new Pos(8.5, 1, 0.5),
                        new BoundingBox(0.6, 0.9, 0.6),
                        profile(EntityType.AXOLOTL), EntityTraversalState.GROUNDED),
                Set.of(PathNode.Movement.WALK, PathNode.Movement.SWIM));

        TestWorld ladder = new TestWorld().floor(-2, 5, -1, 1, 0, Block.STONE);
        Block rung = Block.LADDER.withProperty("facing", "west");
        for (int y = 1; y <= 5; y++) {
            ladder.set(1, y, 0, rung).set(2, y, 0, Block.STONE);
        }
        ladder.set(3, 5, 0, Block.STONE);
        assertRealReplay(request(ladder,
                        new Pos(0.5, 1, 0.5), new Pos(3.5, 6, 0.5), BOX,
                        climbingProfile(), EntityTraversalState.GROUNDED),
                Set.of(PathNode.Movement.CLIMB));

        TestWorld vine = new TestWorld().floor(-2, 5, -1, 1, 0, Block.STONE);
        Block attachedVine = Block.VINE.withProperty("west", "true");
        for (int y = 1; y <= 5; y++) {
            vine.set(1, y, 0, attachedVine).set(2, y, 0, Block.STONE);
        }
        vine.set(3, 5, 0, Block.STONE);
        assertRealReplay(request(vine,
                        new Pos(0.5, 1, 0.5), new Pos(3.5, 6, 0.5), BOX,
                        climbingProfile(), EntityTraversalState.GROUNDED),
                Set.of(PathNode.Movement.CLIMB));

        TestWorld scaffolding = new TestWorld().floor(
                -2, 4, -1, 1, 0, Block.STONE);
        for (int y = 1; y <= 5; y++) {
            scaffolding.set(1, y, 0, Block.SCAFFOLDING);
        }
        scaffolding.set(2, 5, 0, Block.STONE);
        assertRealReplay(request(scaffolding,
                        new Pos(0.5, 1, 0.5), new Pos(2.5, 6, 0.5), BOX,
                        climbingProfile(), EntityTraversalState.GROUNDED),
                Set.of(PathNode.Movement.CLIMB));

        TestWorld door = corridor();
        installDoor(door, 2, Block.OAK_DOOR);
        assertRealReplay(request(door,
                        new Pos(0.5, 1, 0.5), new Pos(4.5, 1, 0.5), BOX,
                        profile(EntityType.VILLAGER),
                        EntityTraversalState.GROUNDED),
                Set.of(PathNode.Movement.WALK));

        TestWorld hazard = new TestWorld().floor(-1, 7, -2, 2, 0, Block.STONE);
        hazard.set(3, 1, 0, Block.FIRE);
        NavigationPlan safe = assertRealReplay(request(hazard,
                        new Pos(0.5, 1, 0.5), new Pos(6.5, 1, 0.5), BOX,
                        profile(EntityType.ZOMBIE),
                        EntityTraversalState.GROUNDED),
                Set.of(PathNode.Movement.WALK));
        assertTrue(safe.nodes().stream().noneMatch(node ->
                        node.graphX() == 3 && node.graphZ() == 0),
                () -> "certified mesh route entered fire: " + safe.nodes());

        TestWorld gap = new TestWorld();
        for (int x = -1; x <= 1; x++) gap.set(x, 0, 0, Block.STONE);
        for (int x = 4; x <= 7; x++) gap.set(x, 0, 0, Block.STONE);
        assertRealReplay(request(gap,
                        new Pos(0.5, 1, 0.5), new Pos(6.5, 1, 0.5), BOX,
                        jumpingProfile(), EntityTraversalState.GROUNDED),
                Set.of(PathNode.Movement.JUMP));

        TestWorld wallClimber = new TestWorld().floor(
                -1, 6, -2, 2, 0, Block.STONE);
        assertRealReplay(request(wallClimber,
                        new Pos(0.5, 1, 0.5), new Pos(5.5, 1, 0.5), BOX,
                        profile(EntityType.SPIDER), EntityTraversalState.GROUNDED),
                Set.of(PathNode.Movement.WALK));
    }

    private static void assertCertifiedReplay(Fixture fixture) {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshPolicy config = SharedMeshPolicy.builder()
                .promotionRequests(2).retentionRequests(1).build();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator((actor, request) -> {
                    searches.incrementAndGet();
                    assertTrue(NavigationPlanParity.requestCompatible(
                            request, fixture.plan), fixture.name);
                    return CompletableFuture.completedFuture(fixture.plan);
                }, config);
        NavigationRequest request = NavigationRequest.builder(
                        new TestWorld(), fixture.plan.start(),
                        fixture.plan.target(), BOX, fixture.plan.profile())
                .maxPathLength(64).nodeSearchRange(64).build();

        coordinator.request(fixture.name + "-seed-1", "target", 4, 0,
                request).plan().join();
        coordinator.request(fixture.name + "-seed-2", "target", 4, 1,
                request).plan().join();
        SharedMeshHandle replay = coordinator.request(
                fixture.name + "-shared", "target", 4, 2, request);

        assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                replay.source(), fixture.name);
        NavigationPlan shared = replay.plan().join();
        assertTrue(NavigationPlanParity.semanticallyEquivalent(
                fixture.plan, shared), fixture.name);
        assertEquals(fixture.plan.nodes().stream()
                        .map(PathNode::movement).toList(),
                shared.nodes().stream().map(PathNode::movement).toList(),
                fixture.name);
        assertEquals(2, searches.get(), fixture.name);
        coordinator.close();
    }

    private static NavigationPlan assertRealReplay(
            NavigationRequest request,
            Set<PathNode.Movement> expectedMovements) {
        EntityPathfinder pathfinder = new EntityPathfinder();
        AtomicInteger searches = new AtomicInteger();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator((actor, submitted) -> {
                    searches.incrementAndGet();
                    return CompletableFuture.completedFuture(
                            NavigationPlan.from(submitted,
                                    pathfinder.findPath(
                                            submitted, SearchControl.NONE)));
                }, SharedMeshPolicy.builder()
                        .promotionRequests(2).retentionRequests(1).build());
        NavigationPlan first = coordinator.request(
                "seed-1", "target", 9, 0, request).plan().join();
        NavigationPlan second = coordinator.request(
                "seed-2", "target", 9, 1, request).plan().join();
        assertTrue(first.usable(), first::toString);
        assertTrue(NavigationPlanParity.semanticallyEquivalent(first, second));
        for (PathNode.Movement expected : expectedMovements) {
            assertTrue(first.nodes().stream().anyMatch(node ->
                            node.movement() == expected),
                    () -> expected + " absent from " + first.nodes());
        }

        SharedMeshHandle replay = coordinator.request(
                "shared", "target", 9, 2, request);
        assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                replay.source(), first.profile().mode().toString());
        NavigationPlan shared = replay.plan().join();
        assertTrue(NavigationPlanParity.semanticallyEquivalent(first, shared),
                () -> "individual=" + first.nodes() + " shared=" + shared.nodes());
        assertEquals(first.nodeCosts(), shared.nodeCosts(),
                "a composed route must price the same cells as its "
                        + "individual pathfinding certificate");
        assertEquals(2, searches.get());
        coordinator.close();
        return shared;
    }

    private static TestWorld corridor() {
        TestWorld world = new TestWorld().floor(-2, 6, -1, 1, 0, Block.STONE);
        for (int x = -2; x <= 6; x++) {
            world.column(x, -1, 1, 4, Block.STONE);
            world.column(x, 1, 1, 4, Block.STONE);
        }
        return world;
    }

    private static void installDoor(TestWorld world, int x, Block door) {
        Block base = door.withProperty("facing", "east")
                .withProperty("hinge", "left")
                .withProperty("open", "false");
        world.set(x, 1, 0, base.withProperty("half", "lower"));
        world.set(x, 2, 0, base.withProperty("half", "upper"));
    }

    /**
     * {@link NavigationPlan#result()} is a lossless carrier of the search that
     * produced it, so the plan-based API reaches the same charged costs the
     * one-shot pathfinder API exposes.
     */
    @Test
    void planResultCarriesEveryChargedCostOfTheSearchItCameFrom() {
        TestWorld world = corridor();
        NavigationRequest request = request(world, new Pos(0.5, 1, 0.5),
                new Pos(6.5, 1, 0.5), BOX, profile(EntityType.ZOMBIE),
                EntityTraversalState.GROUNDED);
        PathResult individual = new EntityPathfinder().findPath(
                request, SearchControl.NONE);
        PathResult replayed = NavigationPlan.from(request, individual).result();

        assertFalse(individual.nodeCosts().isEmpty(),
                "the fixture must price a graph search");
        assertEquals(individual, replayed,
                "a plan must not drop the costs of the search it carries");
        for (PathNode node : replayed.nodes()) {
            assertNotNull(replayed.costAt(
                            node.graphX(), node.graphY(), node.graphZ()),
                    () -> "unpriced route cell " + node);
        }
    }

    /**
     * A shared plan is stitched from decomposed corridor segments. The charged
     * cost of every cell must survive that round trip, so a caller never has
     * to know whether a plan was searched or composed.
     */
    @Test
    void composedSharedRoutesPriceExactlyWhatTheirCertificatePriced() {
        TestWorld world = corridor();
        NavigationRequest request = NavigationRequest.builder(
                        world, new Pos(0.5, 1, 0.5), new Pos(6.5, 1, 0.5),
                        BOX, profile(EntityType.ZOMBIE))
                .maxPathLength(64).nodeSearchRange(64).maxVisitedMultiplier(8)
                .influences(List.of(new NavigationZoneInfluence(
                        new Vec(2, 0, -1), new Vec(4, 3, 1), false, 6, "toll")))
                .build();
        PathResult individual = new EntityPathfinder().findPath(
                request, SearchControl.NONE);
        assertEquals(6, individual.costAt(2, 1, 0).malus(),
                "the fixture must charge an observable toll");

        NavigationPlan shared = assertRealReplay(
                request, Set.of(PathNode.Movement.WALK));
        assertEquals(individual.nodeCosts(), shared.result().nodeCosts(),
                "the mesh dropped or reordered the costs it decomposed");
        assertEquals(6, shared.result().costAt(2, 1, 0).malus());
        assertEquals(6, shared.result().costAt(3, 1, 0).malus());
    }

    private static NavigationRequest request(
            TestWorld world, Pos start, Pos target, BoundingBox box,
            NavigationProfile profile, EntityTraversalState state) {
        return NavigationRequest.builder(world, start, target, box, profile)
                .maxPathLength(64).nodeSearchRange(64)
                .maxVisitedMultiplier(8).entityState(state).build();
    }

    private static TestWorld waterBox(
            int minX, int maxX, int minY, int maxY,
            int minZ, int maxZ) {
        TestWorld world = new TestWorld();
        for (int x = minX; x <= maxX; x++) {
            for (int y = minY; y <= maxY; y++) {
                for (int z = minZ; z <= maxZ; z++) {
                    world.set(x, y, z, Block.WATER);
                }
            }
        }
        return world;
    }

    private static Fixture fixture(
            String name, NavigationProfile profile,
            PathNode.Movement... movements) {
        Pos start = new Pos(0.5, 1, 0.5);
        List<PathNode> nodes = new ArrayList<>();
        for (int index = 0; index < movements.length; index++) {
            double y = switch (movements[index]) {
                case STEP_UP, CLIMB, FLY, SWIM, BREACH -> index + 1;
                default -> 1;
            };
            nodes.add(new PathNode(index + 0.5, y, 0.5,
                    movements[index], index, (int) Math.floor(y), 0));
        }
        Pos target = new Pos(movements.length - 0.5,
                nodes.getLast().y(), 0.5);
        return new Fixture(name, new NavigationPlan(
                start, target, BOX, profile, PathStatus.FOUND,
                nodes, nodes.size(), nodes.size() * 8));
    }

    private static NavigationProfile profile(EntityType type) {
        return BuiltinNavigationProfiles.forEntityType(type);
    }

    private static NavigationProfile climbingProfile() {
        NavigationProfile base = profile(EntityType.ZOMBIE);
        return NavigationProfile.builder(base.mode(), base.mobProfile(), base.groundCapabilities().withClimbables(
                        ClimbableCapabilities.STANDARD)).allowBreaching(base.allowBreaching()).prefersShallowWater(base.prefersShallowWater()).avoidSun(base.avoidSun()).directPathIgnoresFluids(base.directPathIgnoresFluids()).pathToTargetsBelowSurface(base.pathToTargetsBelowSurface()).build();
    }

    private static NavigationProfile jumpingProfile() {
        NavigationProfile base = profile(EntityType.ZOMBIE);
        return NavigationProfile.builder(base.mode(), base.mobProfile(), base.groundCapabilities().withPlatformJump(
                        PlatformJumpCapabilities.builder().maxHorizontalDistance(3).maxRise(1).maxDrop(1).apexClearance(1).build())).allowBreaching(base.allowBreaching()).prefersShallowWater(base.prefersShallowWater()).avoidSun(base.avoidSun()).directPathIgnoresFluids(base.directPathIgnoresFluids()).pathToTargetsBelowSurface(base.pathToTargetsBelowSurface()).build();
    }

    private static NavigationPlan copy(
            NavigationPlan source, List<PathNode> nodes) {
        return new NavigationPlan(
                source.start(), source.target(), source.boundingBox(),
                source.profile(), source.status(), nodes,
                source.visitedNodes(), source.examinedNeighbors());
    }

    private record Fixture(String name, NavigationPlan plan) {
    }
}
