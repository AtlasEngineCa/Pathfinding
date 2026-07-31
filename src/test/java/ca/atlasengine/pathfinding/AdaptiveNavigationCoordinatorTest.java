package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.metrics.NavigationMetrics;import ca.atlasengine.pathfinding.metrics.NavigationMetricsSnapshot;import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.EntityTraversalState;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.adaptive.SharedMeshRetention;
import ca.atlasengine.pathfinding.adaptive.SharedMeshPolicy;
import ca.atlasengine.pathfinding.internal.adaptive.SharedMeshCoordinator;
import ca.atlasengine.pathfinding.adaptive.SharedMeshHandle;
import ca.atlasengine.pathfinding.adaptive.SharedMeshSource;
import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.internal.adaptive.NavigationPlanParity;
import ca.atlasengine.pathfinding.influence.AllowedNavigationAreas;
import ca.atlasengine.pathfinding.influence.BlockAvoidanceInfluence;
import ca.atlasengine.pathfinding.influence.EntityFearInfluence;
import ca.atlasengine.pathfinding.influence.EntitySnapshot;
import ca.atlasengine.pathfinding.influence.InfluenceResult;
import ca.atlasengine.pathfinding.influence.NavigationArea;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.influence.NavigationZoneInfluence;
import ca.atlasengine.pathfinding.influence.ReturnRadiusInfluence;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.lang.reflect.RecordComponent;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

class AdaptiveNavigationCoordinatorTest {
    private static final BoundingBox SMALL = new BoundingBox(0.6, 1.8, 0.6);
    private static final NavigationProfile GROUND =
            BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);

    @Test
    void promotesCompatibleTrafficAndStopsSubmittingIndividualSearches() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshCoordinator coordinator = coordinator(3, searches);
        TestWorld world = new TestWorld();
        List<SharedMeshHandle> handles = new ArrayList<>();

        for (int mob = 0; mob < 3; mob++) {
            SharedMeshHandle handle = coordinator.request(
                    "mob-" + mob, "player", 7, mob,
                    request(world, new Pos(1.5, 1, 1.5),
                            new Pos(20.5, 1, 1.5), SMALL, GROUND));
            assertEquals(SharedMeshSource.INDIVIDUAL_SEARCH,
                    handle.source());
            handles.add(handle);
        }

        SharedMeshHandle shared = coordinator.request(
                "mob-shared", "player", 7, 4,
                request(world, new Pos(1.5, 1, 1.5),
                        new Pos(20.5, 1, 1.5), SMALL, GROUND));

        assertEquals(3, searches.get(),
                "a promoted exact source must not submit another A* search");
        assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                shared.source());
        assertEquals(PathStatus.FOUND, shared.plan().join().status());
        assertEquals(1, coordinator.stats().promotedRegions());
        assertEquals(4, coordinator.stats().actors());

        handles.forEach(SharedMeshHandle::close);
        shared.close();
        coordinator.close();
    }

    @Test
    void separatesBoundingBoxesMovementProfilesAndWorldIdentity() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshCoordinator coordinator = coordinator(2, searches);
        TestWorld firstWorld = new TestWorld();
        TestWorld secondWorld = new TestWorld();
        NavigationProfile flying = BuiltinNavigationProfiles.forEntityType(
                EntityType.BEE);

        coordinator.request("small", "target", 1, 0,
                request(firstWorld, new Pos(1, 1, 1), new Pos(10, 1, 1),
                        SMALL, GROUND));
        coordinator.request("large", "target", 1, 0,
                request(firstWorld, new Pos(1, 1, 1), new Pos(10, 1, 1),
                        new BoundingBox(1.8, 2.7, 1.8), GROUND));
        coordinator.request("flying", "target", 1, 0,
                request(firstWorld, new Pos(1, 1, 1), new Pos(10, 4, 1),
                        SMALL, flying));
        coordinator.request("other-world", "target", 1, 0,
                request(secondWorld, new Pos(1, 1, 1), new Pos(10, 1, 1),
                        SMALL, GROUND));

        assertEquals(4, coordinator.stats().regions(),
                "unsafe capability or world combinations must never share");
        assertEquals(4, searches.get());
        coordinator.close();
    }

    @Test
    void climbableCapabilitiesNeverShareFieldsWithOrdinaryGroundMobs() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshCoordinator coordinator = coordinator(2, searches);
        TestWorld world = new TestWorld();
        NavigationProfile climbing = GROUND.withGroundCapabilities(
                GROUND.groundCapabilities().withClimbables(
                        ClimbableCapabilities.STANDARD));

        coordinator.request("ground", "target", 1, 0,
                request(world, new Pos(1, 1, 1), new Pos(10, 4, 1),
                        SMALL, GROUND));
        coordinator.request("climber", "target", 1, 0,
                request(world, new Pos(1, 1, 1), new Pos(10, 4, 1),
                        SMALL, climbing));

        assertEquals(2, coordinator.stats().regions());
        assertEquals(2, searches.get());
        coordinator.close();
    }

    @Test
    void everySearchSemanticInputPartitionsAdaptiveRegions() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshCoordinator coordinator = coordinator(20, searches);
        TestWorld world = new TestWorld();
        Pos start = new Pos(1.5, 1, 1.5);
        Pos target = new Pos(10.5, 1, 1.5);
        NavigationInfluence influence = (blocks, point, box) ->
                InfluenceResult.penalty(2, "test");
        // The build-height bounds are the traversal snapshot's graph half.
        // minBuildHeight bounds the downward scan that classifies any
        // neighbour and gates the fly evaluator on every cell; maxBuildHeight
        // moves the destination ground normalization resolves, which no
        // target key can see. The snapshot's start-resolving half keys the
        // source instead and is covered by the two tests below it.
        EntityTraversalState shallowWorld = new EntityTraversalState(
                true, false, Set.of(), -32, 319, List.of());
        EntityTraversalState lowCeiling = new EntityTraversalState(
                true, false, Set.of(), -64, 256, List.of());

        List<NavigationRequest> requests = List.of(
                NavigationRequest.builder(world, start, target, SMALL, GROUND)
                        .maxPathLength(64).nodeSearchRange(64).build(),
                NavigationRequest.builder(world, start, target, SMALL, GROUND)
                        .maxPathLength(65).nodeSearchRange(64).build(),
                NavigationRequest.builder(world, start, target, SMALL, GROUND)
                        .maxPathLength(64).nodeSearchRange(65).build(),
                NavigationRequest.builder(world, start, target, SMALL, GROUND)
                        .maxPathLength(64).nodeSearchRange(64)
                        .reachRange(1).build(),
                NavigationRequest.builder(world, start, target, SMALL, GROUND)
                        .maxPathLength(64).nodeSearchRange(64)
                        .maxVisitedMultiplier(2).build(),
                NavigationRequest.builder(world, start, target, SMALL, GROUND)
                        .maxPathLength(64).nodeSearchRange(64)
                        .seaLevel(62).build(),
                NavigationRequest.builder(world, start, target, SMALL, GROUND)
                        .maxPathLength(64).nodeSearchRange(64)
                        .influences(List.of(influence)).build(),
                NavigationRequest.builder(world, start, target, SMALL, GROUND)
                        .maxPathLength(64).nodeSearchRange(64)
                        .entityState(shallowWorld).build(),
                NavigationRequest.builder(world, start, target, SMALL, GROUND)
                        .maxPathLength(64).nodeSearchRange(64)
                        .entityState(lowCeiling).build());

        for (int index = 0; index < requests.size(); index++) {
            coordinator.request("semantic-" + index, "target", 1, 0,
                    requests.get(index));
        }

        assertEquals(requests.size(), coordinator.stats().regions(),
                "cost, reach, traversal, influence, and search semantics "
                        + "must never share a target field");
        assertEquals(requests.size(), searches.get());
        coordinator.close();
    }

    /**
     * The traversal snapshot is split across two keys, so every component of
     * it must be claimed by exactly one of them. A component that is claimed
     * by neither would silently stop separating anything, which is the one
     * failure this design cannot detect at runtime: the parity gate never sees
     * a traversal snapshot, because a plan does not carry one.
     */
    @Test
    void everyTraversalSnapshotComponentIsClaimedByExactlyOneAdaptiveKey() {
        Set<String> regionKeyed = Set.of("minBuildHeight", "maxBuildHeight");
        Set<String> sourceKeyed = Set.of("onGround", "inWater",
                "standableFluids", "pathfindingStartCandidates");
        Set<String> claimed = new TreeSet<>(regionKeyed);
        claimed.addAll(sourceKeyed);
        Set<String> declared = Arrays.stream(
                        EntityTraversalState.class.getRecordComponents())
                .map(RecordComponent::getName)
                .collect(Collectors.toCollection(TreeSet::new));

        assertEquals(claimed, declared,
                "EntityTraversalState changed shape. Decide where the new "
                        + "component belongs before this compiles green: it "
                        + "goes in SharedMeshCoordinator's region key "
                        + "if any evaluator reads it outside getStart, and in "
                        + "the source key only if it can be shown to resolve "
                        + "nothing but the start node");
        assertEquals(regionKeyed.size() + sourceKeyed.size(), claimed.size(),
                "a component cannot key both a region and a source");
    }

    /**
     * The live half of the snapshot resolves nothing but the start node, so a
     * mob that jumps or wades keeps one region instead of allocating another
     * against {@code maximumRegions}. Sharing that region may never let one
     * variant answer another: the two resolve different start nodes from the
     * same exact position, and {@code requestCompatible} cannot tell them
     * apart because it compares the raw request start, which is identical.
     */
    @Test
    void togglingLiveTraversalStateSharesARegionWithoutSharingACertificate() {
        TestWorld world = new TestWorld().floor(0, 14, -3, 3, 0, Block.STONE);
        EntityPathfinder oracle = new EntityPathfinder();
        AtomicInteger searches = new AtomicInteger();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator((actor, submitted) -> {
                    searches.incrementAndGet();
                    return CompletableFuture.completedFuture(
                            NavigationPlan.from(submitted, oracle.findPath(
                                    submitted, SearchControl.NONE)));
                }, SharedMeshPolicy.builder()
                        .promotionRequests(2).retentionRequests(1).build());
        // Five blocks over the floor, so a grounded snapshot rounds to the
        // empty cell the mob occupies while an airborne one walks down to the
        // surface. Both are legal starts and they are not the same node.
        Pos start = new Pos(0.5, 5, 0.5);
        Pos target = new Pos(12.5, 1, 0.5);
        NavigationRequest grounded = airborne(world, start, target, true);
        NavigationRequest jumping = airborne(world, start, target, false);
        NavigationPlan groundedAlone = NavigationPlan.from(
                grounded, oracle.findPath(grounded, SearchControl.NONE));
        NavigationPlan jumpingAlone = NavigationPlan.from(
                jumping, oracle.findPath(jumping, SearchControl.NONE));
        assertFalse(NavigationPlanParity.semanticallyEquivalent(
                        groundedAlone, jumpingAlone),
                "the fixture no longer discriminates: onGround must change "
                        + "the route for this assertion to prove anything");

        try {
            coordinator.request("g1", "target", 1, 0, grounded).plan().join();
            coordinator.request("g2", "target", 1, 1, grounded).plan().join();
            assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    coordinator.request("g3", "target", 1, 2, grounded)
                            .source());

            SharedMeshHandle first = coordinator.request(
                    "j1", "target", 1, 3, jumping);
            assertNotEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    first.source(),
                    "an airborne mob was answered from a grounded mob's "
                            + "certificate for the same exact position");
            assertTrue(NavigationPlanParity.semanticallyEquivalent(
                            jumpingAlone, first.plan().join()),
                    "an airborne mob was not given its own route");
            assertEquals(1, coordinator.stats().regions(),
                    "a live traversal flag must not cost a region slot");

            SharedMeshHandle repeated = coordinator.request(
                    "j2", "target", 1, 4, jumping);
            assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    repeated.source(),
                    "the shared region never certified the airborne source, "
                            + "so the merge bought no reuse at all");
            assertTrue(NavigationPlanParity.semanticallyEquivalent(
                    jumpingAlone, repeated.plan().join()));
            assertEquals(1, coordinator.stats().regions());
            assertEquals(3, searches.get(),
                    "each traversal variant certifies once and no more");
        } finally {
            coordinator.close();
        }
    }

    /**
     * The remaining source-keyed components, for which no ground fixture can
     * change a route: {@code standableFluids} only matters to a mob that
     * stands on one, and start candidates only reach the flying evaluator.
     * They must still never merge two sources, and must still cost no region.
     */
    @Test
    void standableFluidsAndStartCandidatesKeySourcesRatherThanRegions() {
        TestWorld world = new TestWorld().floor(0, 14, -3, 3, 0, Block.STONE);
        AtomicInteger searches = new AtomicInteger();
        SharedMeshCoordinator coordinator = coordinator(2, searches);
        Pos start = new Pos(0.5, 1, 0.5);
        Pos target = new Pos(12.5, 1, 0.5);
        EntityTraversalState plain = EntityTraversalState.GROUNDED;
        EntityTraversalState lavaWalker = new EntityTraversalState(
                true, false, Set.of(Block.LAVA), -64, 319, List.of());
        EntityTraversalState sampled = new EntityTraversalState(
                true, false, Set.of(), -64, 319,
                List.of(new Pos(1, 1, 1)));

        try {
            for (int repeat = 0; repeat < 2; repeat++) {
                coordinator.request("plain-" + repeat, "target", 1, repeat,
                        stateful(world, start, target, plain)).plan().join();
            }
            assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    coordinator.request("plain-shared", "target", 1, 2,
                            stateful(world, start, target, plain)).source());

            // Each remaining snapshot enters an already promoted region. It
            // must still buy its own certificate before anything is replayed
            // to it, and must then be replayed to from that certificate.
            for (EntityTraversalState state : List.of(lavaWalker, sampled)) {
                SharedMeshHandle first = coordinator.request(
                        "first-" + state.standableFluids()
                                + state.pathfindingStartCandidates(),
                        "target", 1, 3,
                        stateful(world, start, target, state));
                assertEquals(
                        SharedMeshSource.PARITY_CERTIFICATION_SEARCH,
                        first.source(),
                        () -> "a snapshot differing only in " + state
                                + " was answered from another source's "
                                + "certificate");
                first.plan().join();
                assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                        coordinator.request("repeat-"
                                        + state.standableFluids()
                                        + state.pathfindingStartCandidates(),
                                "target", 1, 4,
                                stateful(world, start, target, state))
                                .source(),
                        () -> "the shared region never certified " + state
                                + ", so the merge bought no reuse");
            }
            assertEquals(1, coordinator.stats().regions(),
                    "standable fluids and sampled start candidates resolve "
                            + "only the start node, so they must not cost a "
                            + "region each");
            assertEquals(4, searches.get(),
                    "two searches promote the region and each further "
                            + "snapshot certifies its own source exactly once");
        } finally {
            coordinator.close();
        }
    }

    private static NavigationRequest airborne(
            TestWorld world, Pos start, Pos target, boolean onGround) {
        return stateful(world, start, target, new EntityTraversalState(
                onGround, false, Set.of(), -64, 319, List.of()));
    }

    private static NavigationRequest stateful(
            TestWorld world, Pos start, Pos target,
            EntityTraversalState state) {
        return NavigationRequest.builder(world, start, target, SMALL, GROUND)
                .maxPathLength(64).nodeSearchRange(64)
                .entityState(state).build();
    }

    @Test
    void oneRegionSupportsMultipleTargetsAndMovingTargetPositions() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshCoordinator coordinator = coordinator(2, searches);
        TestWorld world = new TestWorld();
        Pos start = new Pos(2.5, 1, 2.5);
        Pos targetA = new Pos(20.5, 1, 2.5);
        Pos targetB = new Pos(22.5, 1, 8.5);

        coordinator.request("a1", "player-a", 3, 0,
                request(world, start, targetA, SMALL, GROUND));
        coordinator.request("b1", "player-b", 3, 1,
                request(world, start, targetB, SMALL, GROUND));

        assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                coordinator.request("a2", "player-a", 3, 2,
                        request(world, start, targetA, SMALL, GROUND)).source());
        assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                coordinator.request("b2", "player-b", 3, 2,
                        request(world, start, targetB, SMALL, GROUND)).source());
        assertEquals(2, coordinator.stats().targets());

        Pos movedA = new Pos(25.5, 1, 2.5);
        SharedMeshHandle firstMoved = coordinator.request(
                "a3", "player-a", 3, 3,
                request(world, start, movedA, SMALL, GROUND));
        assertEquals(SharedMeshSource.PARITY_CERTIFICATION_SEARCH,
                firstMoved.source(), "a new target cell needs one validated edge");
        assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                coordinator.request("a4", "player-a", 3, 4,
                        request(world, start, movedA, SMALL, GROUND)).source());
        assertEquals(3, searches.get());
        coordinator.close();
    }

    /**
     * A pursued entity's position is a live continuous double. Only its block
     * cell reaches the search, so jitter inside one cell must reuse one field
     * and still answer with the plan individual A* returns for that request.
     */
    @Test
    void subBlockTargetMovementReusesOneFieldAndStillMatchesIndividualSearch() {
        TestWorld world = new TestWorld().floor(0, 20, 0, 10, 0, Block.STONE);
        EntityPathfinder oracle = new EntityPathfinder();
        AtomicInteger searches = new AtomicInteger();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator((actor, submitted) -> {
                    searches.incrementAndGet();
                    return CompletableFuture.completedFuture(
                            NavigationPlan.from(submitted, oracle.findPath(
                                    submitted, SearchControl.NONE)));
                }, SharedMeshPolicy.builder()
                        .promotionRequests(2).retentionRequests(1).build());
        Pos start = new Pos(1.5, 1, 5.5);
        List<Pos> jittered = List.of(
                new Pos(12.37, 1, 4.81), new Pos(12.62, 1, 4.12),
                new Pos(12.05, 1, 4.99), new Pos(12.94, 1, 4.5));
        try {
            for (int index = 0; index < 2; index++) {
                coordinator.request("seed-" + index, "player", 1, index,
                        request(world, start, jittered.get(index), SMALL,
                                GROUND)).plan().join();
            }
            for (int index = 2; index < jittered.size(); index++) {
                Pos target = jittered.get(index);
                NavigationRequest request = request(
                        world, start, target, SMALL, GROUND);
                SharedMeshHandle handle = coordinator.request(
                        "pursuer", "player", 1, index, request);
                assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                        handle.source(),
                        "a target that only jittered inside one cell was "
                                + "treated as a new destination");
                NavigationPlan plan = handle.plan().join();
                assertTrue(NavigationPlanParity.semanticallyEquivalent(
                        NavigationPlan.from(request, oracle.findPath(
                                request, SearchControl.NONE)), plan));
                assertEquals(target.x(), plan.target().x());
                assertEquals(target.z(), plan.target().z());
            }
            assertEquals(1, coordinator.stats().targets(),
                    "a jittering target must hold one slot, not one per "
                            + "position it has ever occupied");
            assertEquals(2, searches.get());
        } finally {
            coordinator.close();
        }
    }

    /**
     * The merged destination class is exactly what a search can observe: the
     * block cell plus the {@code floor(y + 0.5)} rounding the amphibious
     * entry point applies. A destination differing in any of those is a
     * different search and never reuses the first one's plan.
     */
    @Test
    void destinationsDifferingInAnySearchObservableWayNeverShareAField() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshCoordinator coordinator = coordinator(2, searches);
        TestWorld world = new TestWorld();
        Pos start = new Pos(1.5, 1, 5.5);
        Pos base = new Pos(12.3, 1.2, 4.7);
        List<Pos> observable = List.of(
                new Pos(13.3, 1.2, 4.7), new Pos(12.3, 2.2, 4.7),
                new Pos(12.3, 1.2, 5.7), new Pos(12.3, 1.7, 4.7));
        try {
            for (int index = 0; index < 2; index++) {
                coordinator.request("seed-" + index, "player", 1, index,
                        request(world, start, base, SMALL, GROUND))
                        .plan().join();
            }
            assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    coordinator.request("same", "player", 1, 2,
                            request(world, start, base, SMALL, GROUND))
                            .source());
            for (Pos target : observable) {
                assertNotEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                        coordinator.request("moved", "player", 1, 3,
                                request(world, start, target, SMALL, GROUND))
                                .source(),
                        () -> "destination " + target + " was answered from "
                                + "the field certified for " + base);
            }
            assertEquals(observable.size() + 1,
                    coordinator.stats().targets());
        } finally {
            coordinator.close();
        }
    }

    /**
     * A moving target must not be able to starve its own pursuers out of a
     * region. A full region reclaims its coldest unoccupied destination
     * instead of dropping the request into untracked planning.
     */
    @Test
    void aFullRegionReclaimsColdDestinationsInsteadOfDroppingPursuers() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshPolicy config = SharedMeshPolicy.builder()
                .promotionRequests(2).retentionRequests(1)
                .maximumTargetsPerRegion(2).build();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator(
                        (actor, request) -> completedPlan(request, searches),
                        config);
        TestWorld world = new TestWorld();
        Pos start = new Pos(1, 1, 1);
        try {
            for (int step = 0; step < 8; step++) {
                SharedMeshHandle handle = coordinator.request(
                        "pursuer", "player", 1, step,
                        request(world, start, new Pos(10 + step, 1, 1),
                                SMALL, GROUND));
                assertNotEquals(SharedMeshSource.UNTRACKED_FALLBACK,
                        handle.source(),
                        "a target that walked away evicted its own pursuer");
                handle.plan().join();
            }
            assertEquals(2, coordinator.stats().targets(),
                    "the hard bound must still hold while reclaiming");
        } finally {
            coordinator.close();
        }
    }

    @Test
    void intermediateCorridorSourceRequiresOneCertificateBeforeReuse() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshPolicy config = SharedMeshPolicy.builder()
                .promotionRequests(2).retentionRequests(1).build();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator((actor, request) -> {
                    searches.incrementAndGet();
                    return CompletableFuture.completedFuture(corridorPlan(request));
                }, config);
        TestWorld world = new TestWorld();
        Pos target = new Pos(8.5, 1, 0.5);
        NavigationRequest full = request(world,
                new Pos(0.5, 1, 0.5), target, SMALL, GROUND);
        coordinator.request("seed-1", "player", 2, 0, full).plan().join();
        coordinator.request("seed-2", "player", 2, 1, full).plan().join();

        NavigationRequest joined = request(world,
                new Pos(4.5, 1, 0.5), target, SMALL, GROUND);
        SharedMeshHandle firstJoin = coordinator.request(
                "joining-mob", "player", 2, 2, joined);

        assertEquals(SharedMeshSource.PARITY_CERTIFICATION_SEARCH,
                firstJoin.source(),
                "an uncertified suffix must not silently replace A*");
        NavigationPlan certified = firstJoin.plan().join();
        SharedMeshHandle shared = coordinator.request(
                "joining-mob-2", "player", 2, 3, joined);
        assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                shared.source());
        NavigationPlan replay = shared.plan().join();
        assertTrue(NavigationPlanParity.semanticallyEquivalent(
                certified, replay));
        assertEquals(4, replay.nodes().getFirst().graphX());
        assertEquals(8, replay.nodes().getLast().graphX());
        assertEquals(3, searches.get(),
                "one A* certificate is required for each exact source");
        coordinator.close();
    }

    @Test
    void cheaperMeshSuffixThatDiffersFromAStarCertificateFallsBack() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator((actor, request) -> {
                    searches.incrementAndGet();
                    return CompletableFuture.completedFuture(
                            actor.toString().startsWith("detour")
                                    ? detourPlan(request)
                                    : corridorPlan(request));
                }, SharedMeshPolicy.builder()
                        .promotionRequests(2).retentionRequests(1).build());
        TestWorld world = new TestWorld();
        Pos target = new Pos(4.5, 1, 0.5);
        NavigationRequest full = request(world,
                new Pos(0.5, 1, 0.5), target, SMALL, GROUND);
        NavigationRequest joined = request(world,
                new Pos(2.5, 1, 0.5), target, SMALL, GROUND);

        coordinator.request("straight-seed", "target", 1, 0, full)
                .plan().join();
        NavigationPlan certificate = coordinator.request(
                "detour-seed", "target", 1, 1, joined).plan().join();
        assertEquals(1, coordinator.stats().promotedRegions());

        SharedMeshHandle fallback = coordinator.request(
                "detour-repeat", "target", 1, 2, joined);
        assertEquals(SharedMeshSource.PARITY_CERTIFICATION_SEARCH,
                fallback.source(),
                "reverse Dijkstra prefers the straight suffix, but it must "
                        + "not replace the certified detour");
        assertTrue(NavigationPlanParity.semanticallyEquivalent(
                certificate, fallback.plan().join()));
        assertEquals(3, searches.get());
        coordinator.close();
    }

    @Test
    void revisionsCleanupTargetsRegionsAndSupersededActorsDeterministically() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshPolicy config = SharedMeshPolicy.builder()
                .promotionRequests(2).retentionRequests(1)
                .targetIdleTicks(2).regionIdleTicks(5)
                .maximumRegions(4).build();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator(
                        (actor, request) -> completedPlan(request, searches), config);
        TestWorld world = new TestWorld();
        NavigationRequest request = request(world,
                new Pos(1.5, 1, 1.5), new Pos(10.5, 1, 1.5), SMALL, GROUND);

        SharedMeshHandle old = coordinator.request(
                "mob", "target", 4, 0, request);
        SharedMeshHandle replacement = coordinator.request(
                "mob", "target", 4, 1, request);
        assertEquals(1, coordinator.stats().actors(),
                "replacing an actor must release its prior membership");
        old.close();
        assertEquals(1, coordinator.stats().actors(),
                "closing a superseded handle must not remove its replacement");

        assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                coordinator.request("second", "target", 4, 1, request).source());
        coordinator.invalidateWorld(world, 5, 2);
        assertEquals(SharedMeshSource.INDIVIDUAL_SEARCH,
                coordinator.request("after-revision", "target", 5, 2, request).source());

        replacement.close();
        coordinator.removeActor("second");
        coordinator.removeActor("after-revision");
        coordinator.tick(4);
        assertEquals(0, coordinator.stats().targets());
        coordinator.tick(8);
        assertEquals(0, coordinator.stats().regions());
        assertEquals(0, coordinator.stats().actors());
        coordinator.close();
    }

    @Test
    void repeatedNotificationOfOneWorldRevisionIsIdempotent() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshCoordinator coordinator = coordinator(2, searches);
        TestWorld world = new TestWorld();
        NavigationRequest request = request(world,
                new Pos(1.5, 1, 1.5), new Pos(10.5, 1, 1.5), SMALL, GROUND);

        coordinator.request("old", "target", 1, 0, request).plan().join();
        coordinator.invalidateWorld(world, 2, 1);
        coordinator.request("new-1", "target", 2, 1, request).plan().join();
        coordinator.invalidateWorld(world, 2, 1);
        coordinator.request("new-2", "target", 2, 2, request).plan().join();

        assertEquals(1, coordinator.stats().promotedRegions(),
                "the second actor observing revision 2 must not erase the "
                        + "first actor's revision-2 certificate");
        assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                coordinator.request("shared", "target", 2, 3, request)
                        .source());
        coordinator.close();
    }

    @Test
    void capacityUsesUntrackedAStarFallbackAndUnloadDropsOnlyMatchingCell() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshPolicy config = SharedMeshPolicy.builder()
                .promotionRequests(2).retentionRequests(1)
                .maximumRegions(1).regionSize(16).verticalRegionSize(16)
                .build();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator(
                        (actor, request) -> completedPlan(request, searches), config);
        TestWorld world = new TestWorld();
        coordinator.request("inside", "target", 1, 0,
                request(world, new Pos(1, 1, 1), new Pos(8, 1, 1),
                        SMALL, GROUND));
        SharedMeshHandle overflow = coordinator.request(
                "outside", "target", 1, 0,
                request(world, new Pos(33, 1, 1), new Pos(40, 1, 1),
                        SMALL, GROUND));

        assertEquals(SharedMeshSource.UNTRACKED_FALLBACK,
                overflow.source());
        assertEquals(1, coordinator.stats().regions());
        assertEquals(1, coordinator.stats().actors());

        coordinator.unloadRegion(world, 0, 0, 0);
        assertEquals(0, coordinator.stats().regions());
        assertEquals(0, coordinator.stats().actors());
        coordinator.close();
    }

    @Test
    void concurrentCompletionsPromoteAtomicallyAndLateResultsCannotResurrectCleanup()
            throws Exception {
        AtomicInteger searches = new AtomicInteger();
        List<CompletableFuture<NavigationPlan>> pending =
                java.util.Collections.synchronizedList(new ArrayList<>());
        SharedMeshPolicy config = SharedMeshPolicy.builder()
                .promotionRequests(4).retentionRequests(1).build();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator((actor, request) -> {
                    searches.incrementAndGet();
                    CompletableFuture<NavigationPlan> result =
                            new CompletableFuture<>();
                    pending.add(result);
                    return result;
                }, config);
        TestWorld world = new TestWorld();
        NavigationRequest request = request(world,
                new Pos(1.5, 1, 1.5), new Pos(30.5, 1, 1.5), SMALL, GROUND);
        int actors = 32;
        CountDownLatch start = new CountDownLatch(1);
        try (var executor = Executors.newFixedThreadPool(8)) {
            List<CompletableFuture<Void>> calls = new ArrayList<>();
            for (int actor = 0; actor < actors; actor++) {
                int id = actor;
                calls.add(CompletableFuture.runAsync(() -> {
                    try {
                        start.await();
                    } catch (InterruptedException exception) {
                        throw new RuntimeException(exception);
                    }
                    coordinator.request("mob-" + id, "player", 9, id, request);
                }, executor));
            }
            start.countDown();
            CompletableFuture.allOf(calls.toArray(CompletableFuture[]::new)).join();
        }
        assertEquals(actors, searches.get());
        NavigationPlan plan = completedPlan(request, new AtomicInteger()).join();
        pending.forEach(future -> future.complete(plan));
        for (CompletableFuture<NavigationPlan> future : pending) future.join();

        SharedMeshHandle shared = coordinator.request(
                "late", "player", 9, 40, request);
        assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                shared.source());
        assertEquals(1, coordinator.stats().promotedRegions());

        coordinator.close();
        assertEquals(0, coordinator.stats().regions());
        assertEquals(0, coordinator.stats().actors());
        assertThrows(IllegalStateException.class, () -> coordinator.request(
                "after-close", "player", 9, 41, request));
    }

    @Test
    void movingTargetHistoryIsHardBounded() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshPolicy config = SharedMeshPolicy.builder()
                .promotionRequests(2).retentionRequests(1)
                .maximumTargetsPerRegion(2).build();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator(
                        (actor, request) -> completedPlan(request, searches), config);
        TestWorld world = new TestWorld();

        coordinator.request("one", "player", 1, 0,
                request(world, new Pos(1, 1, 1), new Pos(10, 1, 1),
                        SMALL, GROUND));
        coordinator.request("two", "player", 1, 1,
                request(world, new Pos(1, 1, 1), new Pos(11, 1, 1),
                        SMALL, GROUND));
        SharedMeshHandle overflow = coordinator.request(
                "three", "player", 1, 2,
                request(world, new Pos(1, 1, 1), new Pos(12, 1, 1),
                        SMALL, GROUND));

        assertEquals(SharedMeshSource.UNTRACKED_FALLBACK,
                overflow.source());
        assertEquals(2, coordinator.stats().targets());
        assertEquals(2, coordinator.stats().actors());
        coordinator.close();
    }

    @Test
    void removingActorCancelsUnfinishedSearchAndPreventsLateRetention() {
        CompletableFuture<NavigationPlan> pending = new CompletableFuture<>();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator((actor, request) -> pending,
                        SharedMeshPolicy.builder()
                                .promotionRequests(2).retentionRequests(1)
                                .build());
        TestWorld world = new TestWorld();
        NavigationRequest request = request(world,
                new Pos(1, 1, 1), new Pos(20, 1, 1), SMALL, GROUND);

        SharedMeshHandle handle = coordinator.request(
                "leaving-mob", "player", 1, 0, request);
        handle.close();

        assertTrue(pending.isCancelled(),
                "despawn cleanup must release queued or running route work");
        assertTrue(handle.plan().isCompletedExceptionally());
        assertEquals(0, coordinator.stats().actors());
        assertEquals(0, coordinator.stats().retainedNodes(),
                "a cancelled late result must not seed the shared graph");
        coordinator.close();
    }

    /**
     * An integrator that rebuilds its influences per entity supplies objects
     * that are equal but never identical. Those requests are interchangeable
     * and must reach the same shared region as one shared instance does.
     */
    @Test
    void equalButDistinctInfluenceObjectsPromoteToOneSharedRegion() {
        for (Supplier<NavigationInfluence> kind : INFLUENCE_KINDS) {
            String name = kind.get().getClass().getSimpleName();
            assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    promoteThirdActor(kind, kind, kind),
                    () -> "equal-but-distinct " + name + " objects never "
                            + "promoted to a shared region");
            Supplier<NavigationInfluence> pinned = single(kind.get());
            assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    promoteThirdActor(pinned, pinned, pinned),
                    () -> "one shared " + name + " instance must still share");
        }
    }

    @Test
    void influencesThatDifferInValueNeverShareARegion() {
        for (Supplier<NavigationInfluence> kind : DISTINGUISHED_KINDS) {
            assertEquals(SharedMeshSource.INDIVIDUAL_SEARCH,
                    promoteThirdActor(INFLUENCE_KINDS.getFirst(),
                            INFLUENCE_KINDS.getFirst(), kind),
                    () -> "a request whose influence differs in value was "
                            + "answered from another region's plan");
        }
    }

    /**
     * A user influence that keeps identity equality never merges. Distinct
     * instances are forced here because the JVM is free to reuse one object
     * for a lambda that captures nothing.
     */
    @Test
    void identityOnlyInfluencesFallBackToIndividualSearches() {
        Supplier<NavigationInfluence> capturing =
                AdaptiveNavigationCoordinatorTest::identityOnly;
        assertEquals(SharedMeshSource.INDIVIDUAL_SEARCH,
                promoteThirdActor(capturing, capturing, capturing));
    }

    private static NavigationInfluence identityOnly() {
        String reason = new StringBuilder("identity").toString();
        return (blocks, point, box) -> InfluenceResult.penalty(2, reason);
    }

    private static final List<Supplier<NavigationInfluence>> INFLUENCE_KINDS =
            List.of(
                    () -> new BlockAvoidanceInfluence(
                            Set.of(Block.FIRE, Block.LAVA), 2, true, 4),
                    () -> new EntityFearInfluence(List.of(new EntitySnapshot(
                            new UUID(1, 2), EntityType.CREEPER, new Vec(9.5, 1, 9.5))),
                            4, 2, 12),
                    () -> AllowedNavigationAreas.of(new NavigationArea(
                            new Vec(-8, -8, -8), new Vec(40, 8, 40), "field")),
                    () -> new NavigationZoneInfluence(new Vec(4, 0, 4),
                            new Vec(6, 4, 6), true, 6, "zone"),
                    () -> new ReturnRadiusInfluence(
                            new Vec(1, 1, 1), new Vec(1, 1, 1), 40));

    /**
     * Each entry differs from {@link #INFLUENCE_KINDS} in exactly one value:
     * a wider scan, a threat that moved, a smaller allowed area, a shifted
     * zone anchor, and a shorter leash.
     */
    private static final List<Supplier<NavigationInfluence>>
            DISTINGUISHED_KINDS = List.of(
                    () -> new BlockAvoidanceInfluence(
                            Set.of(Block.FIRE, Block.LAVA), 3, true, 4),
                    () -> new EntityFearInfluence(List.of(new EntitySnapshot(
                            new UUID(1, 2), EntityType.CREEPER, new Vec(9.5, 1, 9.6))),
                            4, 2, 12),
                    () -> AllowedNavigationAreas.of(new NavigationArea(
                            new Vec(-8, -8, -8), new Vec(39, 8, 40), "field")),
                    () -> new NavigationZoneInfluence(new Vec(5, 0, 4),
                            new Vec(7, 4, 6), true, 6, "zone"),
                    () -> new ReturnRadiusInfluence(
                            new Vec(1, 1, 1), new Vec(1, 1, 1), 39));

    @Test
    void countsEveryDispatchDecisionSoMeshReuseIsDirectlyReadable() {
        AtomicInteger searches = new AtomicInteger();
        NavigationMetrics metrics = new NavigationMetrics();
        SharedMeshPolicy config = SharedMeshPolicy.builder()
                .promotionRequests(2).retentionRequests(1).build();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator(
                        (actor, request) -> completedPlan(request, searches),
                        config, metrics);
        TestWorld world = new TestWorld();
        List<SharedMeshHandle> handles = new ArrayList<>();

        for (int mob = 0; mob < 4; mob++) {
            handles.add(coordinator.request("mob-" + mob, "player", 7, mob,
                    request(world, new Pos(1.5, 1, 1.5),
                            new Pos(20.5, 1, 1.5), SMALL, GROUND)));
        }
        handles.forEach(handle -> handle.plan().join());

        NavigationMetricsSnapshot.AdaptiveSourceCounts sources =
                metrics.snapshot().adaptiveSources();
        assertEquals(4, sources.total());
        assertEquals(2, sources.individualSearch());
        assertEquals(2, sources.sharedTargetField());
        assertEquals(0, sources.parityCertificationSearch());
        assertEquals(0, sources.untrackedFallback());
        assertEquals(0.5, sources.sharedHitRate(), 1.0e-9);
        assertEquals(sources.individualSearch()
                        + sources.parityCertificationSearch(),
                searches.get(),
                "only search-backed dispatches may reach the submitter");

        handles.forEach(SharedMeshHandle::close);
        coordinator.close();
    }

    @Test
    void capacityDrivenFallbackAndCertificationSearchesAreDistinguished() {
        AtomicInteger searches = new AtomicInteger();
        NavigationMetrics metrics = new NavigationMetrics();
        SharedMeshPolicy config = SharedMeshPolicy.builder()
                .promotionRequests(2).retentionRequests(1)
                .maximumRegions(1).regionSize(16).verticalRegionSize(16)
                .build();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator(
                        (actor, request) -> completedPlan(request, searches),
                        config, metrics);
        TestWorld world = new TestWorld();

        coordinator.request("near-1", "player", 1, 0,
                request(world, new Pos(1.5, 1, 1.5),
                        new Pos(20.5, 1, 1.5), SMALL, GROUND)).plan().join();
        coordinator.request("near-2", "player", 1, 1,
                request(world, new Pos(1.5, 1, 1.5),
                        new Pos(20.5, 1, 1.5), SMALL, GROUND)).plan().join();
        SharedMeshHandle certifying = coordinator.request(
                "near-3", "player", 1, 2,
                request(world, new Pos(2.5, 1, 1.5),
                        new Pos(20.5, 1, 1.5), SMALL, GROUND));
        SharedMeshHandle untracked = coordinator.request(
                "far", "player", 1, 3,
                request(world, new Pos(200.5, 1, 200.5),
                        new Pos(220.5, 1, 200.5), SMALL, GROUND));

        assertEquals(SharedMeshSource.PARITY_CERTIFICATION_SEARCH,
                certifying.source());
        assertEquals(SharedMeshSource.UNTRACKED_FALLBACK,
                untracked.source());
        NavigationMetricsSnapshot.AdaptiveSourceCounts sources =
                metrics.snapshot().adaptiveSources();
        assertEquals(2, sources.individualSearch());
        assertEquals(1, sources.parityCertificationSearch());
        assertEquals(1, sources.untrackedFallback());
        assertEquals(0, sources.sharedTargetField());
        assertEquals(0.0, sources.sharedHitRate());

        certifying.close();
        untracked.close();
        coordinator.close();
    }

    /**
     * The mesh endpoint count says nothing about the three per-region plan
     * maps, so a leak confined to them would move no other statistic.
     */
    @Test
    void everyRetainedPlanMapIsReportedSeparatelyAndDrainsWithItsRegion() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshPolicy config = SharedMeshPolicy.builder()
                .promotionRequests(3).retentionRequests(1)
                .targetIdleTicks(4).regionIdleTicks(8).build();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator(
                        (actor, request) -> completedPlan(request, searches),
                        config);
        TestWorld world = new TestWorld();
        List<SharedMeshHandle> handles = new ArrayList<>();

        handles.add(coordinator.request("mob-0", "player", 1, 0,
                request(world, new Pos(1.5, 1, 1.5),
                        new Pos(20.5, 1, 1.5), SMALL, GROUND)));
        handles.add(coordinator.request("mob-1", "player", 1, 1,
                request(world, new Pos(2.5, 1, 1.5),
                        new Pos(20.5, 1, 1.5), SMALL, GROUND)));
        handles.forEach(handle -> handle.plan().join());

        SharedMeshRetention observing =
                coordinator.stats();
        assertEquals(0, observing.promotedRegions());
        assertEquals(2, observing.observedPlans(),
                () -> "observation is invisible before promotion: " + observing);
        assertEquals(2, observing.certifiedPlans());
        assertEquals(0, observing.publishedPlans());
        assertEquals(0, observing.retainedNodes(),
                () -> "no mesh exists yet, so retainedNodes cannot report the "
                        + "routes already held: " + observing);

        handles.add(coordinator.request("mob-2", "player", 1, 2,
                request(world, new Pos(1.5, 1, 1.5),
                        new Pos(20.5, 1, 1.5), SMALL, GROUND)));
        handles.getLast().plan().join();

        SharedMeshRetention promoted =
                coordinator.stats();
        assertEquals(1, promoted.promotedRegions());
        assertEquals(0, promoted.observedPlans(),
                () -> "promotion must hand the observations to the mesh: "
                        + promoted);
        assertTrue(promoted.certifiedPlans() > 0, promoted::toString);
        assertTrue(promoted.publishedPlans() <= promoted.certifiedPlans(),
                () -> "a plan was published without a certificate: " + promoted);
        assertTrue(promoted.retainedNodes() > 0, promoted::toString);

        handles.forEach(SharedMeshHandle::close);
        for (long tick = 3; tick < 40; tick++) coordinator.tick(tick);

        SharedMeshRetention drained =
                coordinator.stats();
        assertEquals(0, drained.regions());
        assertEquals(0, drained.observedPlans(), drained::toString);
        assertEquals(0, drained.certifiedPlans(), drained::toString);
        assertEquals(0, drained.publishedPlans(), drained::toString);
        coordinator.close();
    }

    @Test
    void aStandaloneCoordinatorStillOwnsAPrivateRecorder() {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshCoordinator coordinator = coordinator(2, searches);
        TestWorld world = new TestWorld();

        coordinator.request("mob", "player", 1, 0,
                request(world, new Pos(1.5, 1, 1.5),
                        new Pos(20.5, 1, 1.5), SMALL, GROUND)).plan().join();

        assertEquals(1, coordinator.metrics().snapshot()
                .adaptiveSources().individualSearch());
        coordinator.close();
    }

    /**
     * The reverse Dijkstra a promotion drives must not run inside the monitor
     * the tick thread takes, so the thread that builds it has to be seen
     * without the coordinator lock at least once.
     */
    @Test
    void meshConstructionRunsWithoutHoldingTheCoordinatorMonitor()
            throws Exception {
        Map<Object, CompletableFuture<NavigationPlan>> pending =
                new ConcurrentHashMap<>();
        SharedMeshCoordinator coordinator = pendingCoordinator(pending);
        Watched target = new Watched(coordinator);
        NavigationRequest request = request(new TestWorld(),
                new Pos(1.5, 1, 1.5), new Pos(9.5, 1, 1.5), SMALL, GROUND);
        NavigationPlan plan = completedPlan(request, new AtomicInteger()).join();
        try (var worker = Executors.newSingleThreadExecutor()) {
            coordinator.request("seed", target, 1, 0, request);
            pending.remove("seed").complete(plan);
            coordinator.request("promoting", target, 1, 1, request);
            CompletableFuture<NavigationPlan> promoting =
                    pending.remove("promoting");
            worker.submit(() -> {
                target.watched = Thread.currentThread();
                promoting.complete(plan);
            }).get();

            assertEquals(1, coordinator.stats().promotedRegions());
            assertTrue(target.observations.contains(Boolean.FALSE),
                    "every mesh node was hashed while the promoting thread "
                            + "still owned the coordinator monitor");
        } finally {
            coordinator.close();
        }
    }

    /**
     * A rebuild computed outside the monitor is only published while the
     * region it was computed for is unchanged. Here the destination is dropped
     * mid-build, so the finished composition must be thrown away rather than
     * resurrect a route to a target the region has already forgotten.
     */
    @Test
    void aRebuildIsDiscardedWhenItsRegionChangesWhileItIsBuiltOffTheMonitor()
            throws Exception {
        Map<Object, CompletableFuture<NavigationPlan>> pending =
                new ConcurrentHashMap<>();
        SharedMeshCoordinator coordinator = pendingCoordinator(pending);
        Watched target = new Watched(coordinator);
        target.pause = true;
        NavigationRequest request = request(new TestWorld(),
                new Pos(1.5, 1, 1.5), new Pos(9.5, 1, 1.5), SMALL, GROUND);
        NavigationPlan plan = completedPlan(request, new AtomicInteger()).join();
        try (var worker = Executors.newSingleThreadExecutor()) {
            coordinator.request("seed", target, 1, 0, request);
            pending.remove("seed").complete(plan);
            coordinator.request("promoting", target, 1, 1, request);
            CompletableFuture<NavigationPlan> promoting =
                    pending.remove("promoting");
            Future<?> building = worker.submit(() -> {
                target.watched = Thread.currentThread();
                promoting.complete(plan);
            });

            assertTrue(target.reached.await(5, TimeUnit.SECONDS),
                    "the rebuild never left the coordinator monitor");
            coordinator.removeTarget(target);
            target.release.countDown();
            building.get();

            SharedMeshRetention stats =
                    coordinator.stats();
            assertEquals(0, stats.targets(), stats::toString);
            assertEquals(0, stats.publishedPlans(),
                    () -> "a rebuild computed before the destination was "
                            + "forgotten was published anyway: " + stats);
            assertNotEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    coordinator.request("late", target, 1, 2, request).source());
        } finally {
            coordinator.close();
        }
    }

    /**
     * Certifications completing on many threads while a tick thread contends
     * for the same coordinator may never hand a follower a plan it could tell
     * apart from its own search, so every shared answer is re-derived here.
     */
    @Test
    void racingCertificationsNeverPublishAPlanThatDivergesFromIndividualSearch()
            throws Exception {
        TestWorld world = new TestWorld().floor(0, 15, 0, 15, 0, Block.STONE);
        for (int index = 0; index < 12; index++) {
            world.column(2 + index % 12, 3 + (index * 5) % 11, 1, 3, Block.STONE);
        }
        Map<Object, CompletableFuture<NavigationPlan>> pending =
                new ConcurrentHashMap<>();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator((actor, submitted) -> {
                    CompletableFuture<NavigationPlan> future =
                            new CompletableFuture<>();
                    pending.put(actor, future);
                    return future;
                }, SharedMeshPolicy.builder()
                        .promotionRequests(4).retentionRequests(1).build());
        List<Pos> starts = List.of(new Pos(0.5, 1, 0.5), new Pos(0.5, 1, 15.5),
                new Pos(15.5, 1, 0.5), new Pos(1.5, 1, 8.5));
        Pos destination = new Pos(14.5, 1, 14.5);
        AtomicInteger shared = new AtomicInteger();
        AtomicInteger divergences = new AtomicInteger();
        CountDownLatch stop = new CountDownLatch(1);
        Thread ticker = new Thread(() -> {
            while (stop.getCount() > 0) coordinator.tick(0);
        });
        ticker.setDaemon(true);
        ticker.start();
        try (var pool = Executors.newFixedThreadPool(4)) {
            List<CompletableFuture<Void>> calls = new ArrayList<>();
            for (int worker = 0; worker < 4; worker++) {
                int lane = worker;
                calls.add(CompletableFuture.runAsync(() -> {
                    EntityPathfinder oracle = new EntityPathfinder();
                    for (int round = 0; round < 24; round++) {
                        NavigationRequest request = request(world,
                                starts.get((lane + round) % starts.size()),
                                destination, SMALL, GROUND);
                        Object actor = "racer-" + lane + '-' + round;
                        SharedMeshHandle handle = coordinator.request(
                                actor, "player", 1, round, request);
                        CompletableFuture<NavigationPlan> search =
                                pending.remove(actor);
                        if (search != null) {
                            search.complete(NavigationPlan.from(request,
                                    oracle.findPath(request, SearchControl.NONE)));
                        }
                        NavigationPlan answer = handle.plan().join();
                        if (handle.source()
                                != SharedMeshSource.SHARED_TARGET_FIELD) {
                            continue;
                        }
                        shared.incrementAndGet();
                        if (!NavigationPlanParity.semanticallyEquivalent(
                                NavigationPlan.from(request, oracle.findPath(
                                        request, SearchControl.NONE)), answer)) {
                            divergences.incrementAndGet();
                        }
                    }
                }, pool));
            }
            CompletableFuture.allOf(calls.toArray(CompletableFuture[]::new))
                    .join();
        } finally {
            stop.countDown();
            ticker.join(5000);
            coordinator.close();
        }
        assertEquals(0, divergences.get(),
                "a racing rebuild published a plan an individual search "
                        + "would not have returned");
        assertTrue(shared.get() > 0, "no shared plan was ever published");
    }

    private static SharedMeshCoordinator pendingCoordinator(
            Map<Object, CompletableFuture<NavigationPlan>> pending) {
        return new SharedMeshCoordinator((actor, request) -> {
            CompletableFuture<NavigationPlan> future = new CompletableFuture<>();
            pending.put(actor, future);
            return future;
        }, SharedMeshPolicy.builder()
                .promotionRequests(2).retentionRequests(1).build());
    }

    /**
     * A target key the coordinator hashes while it walks its mesh, which
     * records whether the rebuilding thread still owns the monitor and can
     * hold that thread outside it while the test changes the region.
     */
    private static final class Watched {
        final Set<Boolean> observations = ConcurrentHashMap.newKeySet();
        final CountDownLatch reached = new CountDownLatch(1);
        final CountDownLatch release = new CountDownLatch(1);
        private final Object monitor;
        private final AtomicBoolean armed = new AtomicBoolean(true);
        volatile Thread watched;
        volatile boolean pause;

        Watched(Object monitor) {
            this.monitor = monitor;
        }

        @Override
        public int hashCode() {
            observe();
            return 31;
        }

        @Override
        public boolean equals(Object other) {
            observe();
            return other instanceof Watched;
        }

        private void observe() {
            if (Thread.currentThread() != watched) return;
            boolean held = Thread.holdsLock(monitor);
            observations.add(held);
            if (held || !pause || !armed.compareAndSet(true, false)) return;
            reached.countDown();
            try {
                release.await(5, TimeUnit.SECONDS);
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
            }
        }
    }

    private static Supplier<NavigationInfluence> single(
            NavigationInfluence influence) {
        return () -> influence;
    }

    private static SharedMeshSource promoteThirdActor(
            Supplier<NavigationInfluence> first,
            Supplier<NavigationInfluence> second,
            Supplier<NavigationInfluence> third) {
        AtomicInteger searches = new AtomicInteger();
        SharedMeshCoordinator coordinator = coordinator(2, searches);
        TestWorld world = new TestWorld();
        try {
            coordinator.request("seed-1", "target", 1, 0,
                    influenced(world, first.get())).plan().join();
            coordinator.request("seed-2", "target", 1, 1,
                    influenced(world, second.get())).plan().join();
            return coordinator.request("third", "target", 1, 2,
                    influenced(world, third.get())).source();
        } finally {
            coordinator.close();
        }
    }

    private static NavigationRequest influenced(
            TestWorld world, NavigationInfluence influence) {
        return NavigationRequest.builder(world, new Pos(1.5, 1, 1.5),
                        new Pos(10.5, 1, 1.5), SMALL, GROUND)
                .maxPathLength(128).nodeSearchRange(128)
                .influences(List.of(influence)).build();
    }

    private static SharedMeshCoordinator coordinator(
            int threshold, AtomicInteger searches) {
        SharedMeshPolicy config = SharedMeshPolicy.builder()
                .promotionRequests(threshold).retentionRequests(1).build();
        return new SharedMeshCoordinator(
                (actor, request) -> completedPlan(request, searches), config);
    }

    private static CompletableFuture<NavigationPlan> completedPlan(
            NavigationRequest request, AtomicInteger searches) {
        searches.incrementAndGet();
        Pos start = request.start().asPos();
        Pos target = request.target().asPos();
        return CompletableFuture.completedFuture(new NavigationPlan(
                start, target, request.boundingBox(), request.profile(),
                PathStatus.FOUND,
                List.of(new PathNode(start.x(), start.y(), start.z(),
                                PathNode.Movement.WALK,
                                start.blockX(), start.blockY(), start.blockZ()),
                        new PathNode(target.x(), target.y(), target.z(),
                                PathNode.Movement.WALK,
                                target.blockX(), target.blockY(), target.blockZ())),
                2, 8));
    }

    private static NavigationPlan corridorPlan(NavigationRequest request) {
        List<PathNode> nodes = new ArrayList<>();
        int startX = request.start().blockX();
        int targetX = request.target().blockX();
        for (int x = startX; x <= targetX; x++) {
            nodes.add(new PathNode(x + 0.5, request.start().y(),
                    request.start().z(), PathNode.Movement.WALK,
                    x, request.start().blockY(), request.start().blockZ()));
        }
        return new NavigationPlan(
                request.start(), request.target(), request.boundingBox(),
                request.profile(), PathStatus.FOUND, nodes,
                nodes.size(), nodes.size() * 8);
    }

    private static NavigationPlan detourPlan(NavigationRequest request) {
        List<PathNode> nodes = List.of(
                new PathNode(2.5, 1, 0.5, PathNode.Movement.WALK, 2, 1, 0),
                new PathNode(2.5, 1, 2.5, PathNode.Movement.WALK, 2, 1, 2),
                new PathNode(4.5, 1, 2.5, PathNode.Movement.WALK, 4, 1, 2),
                new PathNode(4.5, 1, 0.5, PathNode.Movement.WALK, 4, 1, 0));
        return new NavigationPlan(
                request.start(), request.target(), request.boundingBox(),
                request.profile(), PathStatus.FOUND, nodes, 4, 32);
    }

    private static NavigationRequest request(
            TestWorld world, Pos start, Pos target,
            BoundingBox box, NavigationProfile profile) {
        return NavigationRequest.builder(world, start, target, box, profile)
                .maxPathLength(128).nodeSearchRange(128).build();
    }
}
