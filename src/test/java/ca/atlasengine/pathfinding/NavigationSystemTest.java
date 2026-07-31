package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.metrics.NavigationMetricsSnapshot;import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.adaptive.SharedMeshPursuit;
import ca.atlasengine.pathfinding.adaptive.SharedMeshPolicy;
import ca.atlasengine.pathfinding.adaptive.SharedMeshHandle;
import ca.atlasengine.pathfinding.adaptive.SharedMeshSource;
import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.internal.adaptive.NavigationPlanParity;
import ca.atlasengine.pathfinding.adaptive.SharedMeshNavigation;
import ca.atlasengine.pathfinding.adaptive.SharedMeshOptions;
import ca.atlasengine.pathfinding.adaptive.SharedMeshRequest;
import ca.atlasengine.pathfinding.adaptive.SharedMeshStatus;
import net.minestom.server.coordinate.ChunkRange;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
import net.minestom.testing.Env;
import net.minestom.testing.EnvTest;
import org.junit.jupiter.api.Test;

import java.time.Duration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

@EnvTest
class NavigationSystemTest {
    @Test
    void primaryFacadeNavigatesBuiltinEntityEndToEnd(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 1,
                (x, z) -> instance.loadChunk(x, z).join());
        EntityCreature entity = new EntityCreature(EntityType.ZOMBIE);
        entity.setInstance(instance, new Pos(0.5, 40, 0.5)).join();

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1)
                .queueCapacity(8)
                .movementPerTick(0.2)
                .build()) {
            EntityNavigationController controller =
                    navigation.controller(entity);
            controller.moveTo(new Pos(8.5, 40, 0.5));

            org.junit.jupiter.api.Assertions.assertTimeoutPreemptively(
                    Duration.ofSeconds(5), () -> {
                        for (int tick = 0; tick < 600; tick++) {
                            controller.tick();
                            env.tick();
                            if (controller.state()
                                    == NavigationState.COMPLETED) return;
                            if (controller.state()
                                    == NavigationState.COMPUTING) {
                                Thread.sleep(1);
                            }
                        }
                    });

            assertEquals(NavigationState.COMPLETED, controller.state());
            assertTrue(entity.getPosition().distance(
                    new Pos(8.5, 40, 0.5)) < 1.5);
            assertEquals(1, navigation.options().parallelism());
        }
    }

    @Test
    void builderRejectsUnsafeWorkerAndMovementSettings() {
        assertThrows(IllegalArgumentException.class,
                () -> NavigationSystem.builder().parallelism(0));
        assertThrows(IllegalArgumentException.class,
                () -> NavigationSystem.builder().queueCapacity(0));
        assertThrows(IllegalArgumentException.class,
                () -> NavigationSystem.builder()
                        .maximumDeferredControllerRequests(0));
        assertThrows(IllegalArgumentException.class,
                () -> NavigationSystem.builder()
                        .maximumConcurrentControllerSearches(0));
        assertThrows(IllegalArgumentException.class,
                () -> NavigationSystem.builder().minimumSearchStallTicks(0));
        assertThrows(IllegalArgumentException.class,
                () -> NavigationSystem.builder().shedBackoffTicks(0));
        assertThrows(IllegalArgumentException.class,
                () -> NavigationSystem.builder().movementPerTick(0));
        assertThrows(IllegalArgumentException.class,
                () -> NavigationSystem.builder().sharedMesh(null));
    }

    @Test
    void controllerSchedulingSettingsAreObservable() {
        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(4)
                .maximumConcurrentControllerSearches(2)
                .maximumDeferredControllerRequests(12_345)
                .minimumSearchStallTicks(17)
                .shedBackoffTicks(9)
                .build()) {
            assertEquals(2, navigation.options().maximumConcurrentControllerSearches());
            assertEquals(12_345,
                    navigation.options().maximumDeferredControllerRequests());
            assertEquals(17, navigation.options().minimumSearchStallTicks());
            assertEquals(9, navigation.options().shedBackoffTicks());
            assertEquals(0, navigation.activeControllerSearches());
            assertEquals(0, navigation.deferredControllerSearches());
        }
    }

    @Test
    void computedPlanCanBeFedDirectlyToFollowerWithoutAnotherSearch(Env env)
            throws InterruptedException {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 1,
                (x, z) -> instance.loadChunk(x, z).join());
        EntityCreature entity = new EntityCreature(EntityType.ZOMBIE);
        entity.setInstance(instance, new Pos(0.5, 40, 0.5)).join();
        NavigationProfile profile = BuiltinNavigationProfiles.forEntity(entity);

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).movementPerTick(0.2).build()) {
            NavigationRequest request = NavigationRequest.builder(instance,
                            entity.getPosition(), new Pos(8.5, 40, 0.5),
                            entity.getBoundingBox(), profile)
                    .maxPathLength(32)
                    .build();
            NavigationPlan plan = navigation.plan(request).join();
            assertTrue(plan.usable(), plan::toString);
            EntityNavigationController controller = navigation.controller(entity);

            controller.follow(plan);
            for (int attempt = 0;
                 navigation.activeSearches() != 0 && attempt < 100; attempt++) {
                Thread.sleep(1);
            }
            assertEquals(0, navigation.activeSearches());
            assertEquals(0, navigation.queuedSearches());
            org.junit.jupiter.api.Assertions.assertTimeoutPreemptively(
                    Duration.ofSeconds(5), () -> {
                        for (int tick = 0; tick < 600; tick++) {
                            controller.tick();
                            env.tick();
                            if (controller.state() == NavigationState.COMPLETED) return;
                        }
                    });

            assertEquals(NavigationState.COMPLETED, controller.state());
            assertTrue(entity.getPosition().distance(
                    new Pos(8.5, 40, 0.5)) < 1.5);
            assertEquals(1, controller.generation());
        }
    }

    @Test
    void adaptiveFacadePromotesRealAStarPlansAndReturnsFollowerReadyReuse(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 1,
                (x, z) -> instance.loadChunk(x, z).join());
        NavigationProfile profile =
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
        NavigationRequest request = NavigationRequest.builder(instance,
                        new Pos(0.5, 40, 0.5), new Pos(12.5, 40, 0.5),
                        EntityType.ZOMBIE.boundingBox(), profile)
                .maxPathLength(32).build();
        SharedMeshPolicy adaptive = SharedMeshPolicy.builder()
                .promotionRequests(2).retentionRequests(1).build();

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8)
                .sharedMesh(SharedMeshOptions.enabledWith(adaptive)).build()) {
            SharedMeshHandle first = navigation.sharedMesh().plan(SharedMeshRequest.builder(request).actor("mob-1").target("player").worldRevision(1).currentTick(0).build());
            SharedMeshHandle second = navigation.sharedMesh().plan(SharedMeshRequest.builder(request).actor("mob-2").target("player").worldRevision(1).currentTick(1).build());
            assertTrue(first.plan().join().usable());
            assertTrue(second.plan().join().usable());

            SharedMeshHandle shared = navigation.sharedMesh().plan(SharedMeshRequest.builder(request).actor("mob-3").target("player").worldRevision(1).currentTick(2).build());
            assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    shared.source());
            NavigationPlan replay = shared.plan().join();
            assertTrue(replay.usable());
            assertTrue(request.target().distance(replay.target()) < 1.0e-9);
            assertEquals(1, navigation.sharedMesh().status()
                    .regions().promotedRegions());

            first.close();
            second.close();
            shared.close();
        }
    }

    @Test
    void adaptiveThreeDimensionalRouteIsReusedByLiveFlyingFollower(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 2,
                (x, z) -> instance.loadChunk(x, z).join());
        EntityCreature bee = new EntityCreature(EntityType.BEE);
        Pos start = new Pos(0.5, 45, 0.5);
        Pos target = new Pos(12.5, 49, 5.5);
        bee.setInstance(instance, start).join();
        NavigationProfile profile = BuiltinNavigationProfiles.forEntity(bee);
        NavigationRequest request = NavigationRequest.builder(instance,
                        start, target, bee.getBoundingBox(), profile)
                .maxPathLength(48).nodeSearchRange(48).build();
        SharedMeshPolicy adaptive = SharedMeshPolicy.builder()
                .promotionRequests(2).retentionRequests(1).build();

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8)
                .sharedMesh(SharedMeshOptions.enabledWith(adaptive)).build()) {
            navigation.sharedMesh().plan(SharedMeshRequest.builder(request).actor("bee-seed-1").target("player").worldRevision(1).currentTick(0).build())
                    .plan().join();
            navigation.sharedMesh().plan(SharedMeshRequest.builder(request).actor("bee-seed-2").target("player").worldRevision(1).currentTick(1).build())
                    .plan().join();
            SharedMeshHandle shared = navigation.sharedMesh().plan(SharedMeshRequest.builder(request).actor(bee.getUuid()).target("player").worldRevision(1).currentTick(2).build());
            assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    shared.source());
            NavigationPlan plan = shared.plan().join();
            assertTrue(plan.nodes().stream().anyMatch(node ->
                    node.graphY() != start.blockY()
                            || node.graphZ() != start.blockZ()));
            assertTrue(plan.nodes().stream().allMatch(node ->
                    node.movement() == PathNode.Movement.FLY));

            EntityNavigationController controller = navigation.controller(bee);
            controller.follow(plan);
            org.junit.jupiter.api.Assertions.assertTimeoutPreemptively(
                    Duration.ofSeconds(5), () -> {
                        for (int tick = 0; tick < 800; tick++) {
                            controller.tick();
                            env.tick();
                            if (controller.state() == NavigationState.COMPLETED) return;
                        }
                    });

            assertEquals(NavigationState.COMPLETED, controller.state());
            assertTrue(bee.getPosition().distance(target) < 1.5,
                    () -> "flying shared follower ended at " + bee.getPosition());
            shared.close();
        }
    }

    @Test
    void systemSnapshotCoversSearchesLatencyAndAdaptiveReuseAtOnce(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 1,
                (x, z) -> instance.loadChunk(x, z).join());
        NavigationProfile profile =
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
        NavigationRequest request = NavigationRequest.builder(instance,
                        new Pos(0.5, 40, 0.5), new Pos(12.5, 40, 0.5),
                        EntityType.ZOMBIE.boundingBox(), profile)
                .maxPathLength(32).build();
        SharedMeshPolicy adaptive = SharedMeshPolicy.builder()
                .promotionRequests(2).retentionRequests(1).build();

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8)
                .sharedMesh(SharedMeshOptions.enabledWith(adaptive)).build()) {
            NavigationMetricsSnapshot before = navigation.metricsSnapshot();
            assertEquals(0, before.searches().submitted());

            SharedMeshHandle first = navigation.sharedMesh().plan(SharedMeshRequest.builder(request).actor("mob-1").target("player").worldRevision(1).currentTick(0).build());
            SharedMeshHandle second = navigation.sharedMesh().plan(SharedMeshRequest.builder(request).actor("mob-2").target("player").worldRevision(1).currentTick(1).build());
            first.plan().join();
            second.plan().join();
            SharedMeshHandle shared = navigation.sharedMesh().plan(SharedMeshRequest.builder(request).actor("mob-3").target("player").worldRevision(1).currentTick(2).build());
            shared.plan().join();

            NavigationMetricsSnapshot after = navigation.metricsSnapshot();
            var sources = after.adaptiveSources();
            assertEquals(3, sources.total());
            assertEquals(1, sources.sharedTargetField());
            assertTrue(sources.sharedHitRate() > 0.3
                    && sources.sharedHitRate() < 0.34, sources::toString);
            assertEquals(sources.individualSearch()
                            + sources.parityCertificationSearch()
                            + sources.untrackedFallback(),
                    after.searches().submitted(),
                    "only search-backed dispatches enter the worker pool");
            assertTrue(after.searches().submitted()
                    >= after.searches().terminated());
            assertTrue(after.latency().count()
                    <= after.searches().completed());
            assertTrue(after.latency().maximumMicros() > 0);
            assertTrue(after.latency().p99Micros()
                    >= after.latency().p50Micros());
            assertEquals(1, after.queue().parallelism());
            assertEquals(8, after.queue().queueCapacity(),
                    "the builder's queue bound must be readable from a "
                            + "snapshot, not only known out of band");
            assertEquals(8, navigation.options().queueCapacity());
            assertEquals(0, after.stalePlans());
            assertTrue(after.elapsedSecondsSince(before) >= 0);
            assertEquals(2, after.searches().submitted()
                            - before.searches().submitted(),
                    "counters are cumulative, so a window is a difference");

            first.close();
            second.close();
            shared.close();
        }
    }

    @Test
    void controllerOutcomesAreCountedWithoutRetainingAnyController(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 1,
                (x, z) -> instance.loadChunk(x, z).join());
        EntityCreature entity = new EntityCreature(EntityType.ZOMBIE);
        entity.setInstance(instance, new Pos(0.5, 40, 0.5)).join();

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).movementPerTick(0.2).build()) {
            EntityNavigationController controller =
                    navigation.controller(entity);
            controller.moveTo(new Pos(6.5, 40, 0.5));
            org.junit.jupiter.api.Assertions.assertTimeoutPreemptively(
                    Duration.ofSeconds(5), () -> {
                        for (int tick = 0; tick < 600; tick++) {
                            controller.tick();
                            env.tick();
                            if (controller.state()
                                    == NavigationState.COMPLETED) return;
                            if (controller.state()
                                    == NavigationState.COMPUTING) {
                                Thread.sleep(1);
                            }
                        }
                    });
            assertEquals(NavigationState.COMPLETED, controller.state());

            var outcomes = navigation.metricsSnapshot().controllerOutcomes();
            assertEquals(1, outcomes.completed());
            assertEquals(1, outcomes.total());
            assertEquals(0.0, outcomes.stallRate());
            assertEquals(0.0, outcomes.travelStallRate());
            assertTrue(outcomes.travelling() <= outcomes.total());

            controller.close();
            assertEquals(1, navigation.metricsSnapshot()
                            .controllerOutcomes().total(),
                    "closing a finished navigation is not a second outcome");

            EntityNavigationController unused = navigation.controller(entity);
            unused.close();
            assertEquals(1, navigation.metricsSnapshot()
                            .controllerOutcomes().total(),
                    "a controller that never navigated has no outcome");

            EntityNavigationController abandoned =
                    navigation.controller(entity);
            abandoned.moveTo(new Pos(9.5, 40, 0.5));
            abandoned.close();
            assertEquals(1, navigation.metricsSnapshot()
                    .controllerOutcomes().cancelled());

            EntityCreature detached = new EntityCreature(EntityType.ZOMBIE);
            EntityNavigationController orphan = navigation.controller(detached);
            orphan.moveTo(new Pos(1.5, 40, 1.5));
            assertEquals(NavigationState.FAILED, orphan.state());
            assertEquals(1, navigation.metricsSnapshot()
                    .controllerOutcomes().failed());
            assertEquals(3, navigation.metricsSnapshot()
                    .controllerOutcomes().total());
        }
    }

    @Test
    void automaticEntityHandleFollowsMovingTargetAndCleansUpOnRemoval(Env env)
            throws InterruptedException {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 2,
                (x, z) -> instance.loadChunk(x, z).join());
        EntityCreature actor = new EntityCreature(EntityType.ZOMBIE);
        EntityCreature seedActorOne = new EntityCreature(EntityType.ZOMBIE);
        EntityCreature seedActorTwo = new EntityCreature(EntityType.ZOMBIE);
        EntityCreature target = new EntityCreature(EntityType.ZOMBIE);
        Pos start = new Pos(0.5, 40, 0.5);
        Pos destination = new Pos(10.5, 40, 0.5);
        actor.setInstance(instance, start).join();
        seedActorOne.setInstance(instance, start).join();
        seedActorTwo.setInstance(instance, start).join();
        target.setInstance(instance, destination).join();
        for (int settle = 0; settle < 5; settle++) env.tick();
        SharedMeshPolicy adaptive = SharedMeshPolicy.builder()
                .promotionRequests(2).retentionRequests(1).build();

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8)
                .sharedMesh(SharedMeshOptions.enabledWith(adaptive)).build()) {
            SharedMeshPursuit seedOne = navigation.sharedMesh().pursue(
                    seedActorOne, target, 1, 0);
            SharedMeshPursuit seedTwo = navigation.sharedMesh().pursue(
                    seedActorTwo, target, 1, 1);
            for (int tick = 2; tick < 200
                    && navigation.sharedMesh().status().regions()
                    .promotedRegions() == 0; tick++) {
                seedOne.tick(1, tick);
                seedTwo.tick(1, tick);
                env.tick();
                if (navigation.activeSearches() != 0
                        || navigation.queuedSearches() != 0) Thread.sleep(1);
            }
            assertEquals(1, navigation.sharedMesh().status()
                    .regions().promotedRegions());

            SharedMeshPursuit automatic = navigation.sharedMesh().pursue(
                    actor, target, 1, 2);
            assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    automatic.source(), navigation.sharedMesh().status()
                            .regions()::toString);
            org.junit.jupiter.api.Assertions.assertTimeoutPreemptively(
                    Duration.ofSeconds(5), () -> {
                        for (int tick = 3; tick < 700; tick++) {
                            automatic.tick(1, tick);
                            env.tick();
                            if (automatic.state()
                                    == NavigationState.COMPLETED) return;
                        }
                    });
            assertEquals(NavigationState.COMPLETED, automatic.state());

            target.refreshPosition(new Pos(12.5, 40, 0.5));
            automatic.tick(1, 701);
            assertEquals(3, navigation.sharedMesh().status().regions().actors(),
                    "moving a target must replace, not duplicate, membership");

            automatic.tick(2, 702);
            assertEquals(SharedMeshSource.INDIVIDUAL_SEARCH,
                    automatic.source(),
                    "a world revision must invalidate and resubmit even when "
                            + "the target remains in the same cell");
            assertEquals(0, navigation.sharedMesh().status().regions()
                    .promotedRegions());

            actor.remove();
            automatic.tick(2, 703);
            assertTrue(automatic.closed());
            assertEquals(2, navigation.sharedMesh().status().regions().actors());
            seedOne.close();
            seedTwo.close();
            assertEquals(0, navigation.sharedMesh().status().regions().actors());
        }
    }

    @Test
    void sharedMeshIsInertUntilTheBuilderOptsIn(Env env) {
        Instance instance = flatInstance(env);
        NavigationRequest request = crossingRequest(instance);

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8)
                .sharedMesh(new SharedMeshOptions(false, seeding())).build()) {
            SharedMeshNavigation mesh = navigation.sharedMesh();
            assertFalse(mesh.enabled(),
                    "the mesh must never be reached without an opt-in");

            for (int actor = 0; actor < 4; actor++) {
                SharedMeshHandle handle = mesh.plan(meshRequest(
                        request, "mob-" + actor, actor,
                        NavigationStrategy.PREFER_SHARED));
                assertEquals(SharedMeshSource.UNSHARED_SEARCH,
                        handle.source());
                assertTrue(handle.plan().join().usable());
                handle.close();
            }

            SharedMeshStatus status = mesh.status();
            assertEquals(0, status.meshRequests());
            assertEquals(4, status.bypassedRequests());
            assertEquals(0.0, status.sharedHitRate());
            assertEquals(0, status.regions().regions(),
                    () -> "a disabled mesh retained state: " + status);
            assertEquals(4, navigation.metricsSnapshot()
                    .adaptiveSources().unsharedSearch());
        }
    }

    @Test
    void forcingIndividualKeepsOneRequestOffAWarmMesh(Env env) {
        Instance instance = flatInstance(env);
        NavigationRequest request = crossingRequest(instance);

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8)
                .sharedMesh(SharedMeshOptions.enabledWith(seeding())).build()) {
            SharedMeshNavigation mesh = navigation.sharedMesh();
            mesh.plan(meshRequest(request, "seed-1", 0,
                    NavigationStrategy.PREFER_SHARED)).plan().join();
            mesh.plan(meshRequest(request, "seed-2", 1,
                    NavigationStrategy.PREFER_SHARED)).plan().join();

            SharedMeshHandle forced = mesh.plan(meshRequest(
                    request, "forced", 2,
                    NavigationStrategy.INDIVIDUAL_ONLY));
            assertEquals(SharedMeshSource.UNSHARED_SEARCH,
                    forced.source());
            assertTrue(forced.plan().join().usable());

            SharedMeshHandle shared = mesh.plan(meshRequest(
                    request, "shared", 3, NavigationStrategy.PREFER_SHARED));
            assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    shared.source(),
                    "forcing one request must not cool the mesh");

            SharedMeshStatus status = mesh.status();
            assertEquals(3, status.meshRequests());
            assertEquals(1, status.sharedPlans());
            assertEquals(1, status.bypassedRequests());
            assertFalse(status.regionsExhausted());
            forced.close();
            shared.close();
        }
    }

    @Test
    void preferringSharedNeverBypassesTheParityGate(Env env) {
        Instance instance = flatInstance(env);
        NavigationRequest request = crossingRequest(instance);

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8)
                .sharedMesh(SharedMeshOptions.enabledWith(seeding())).build()) {
            SharedMeshNavigation mesh = navigation.sharedMesh();

            SharedMeshHandle cold = mesh.plan(meshRequest(
                    request, "seed-1", 0, NavigationStrategy.PREFER_SHARED));
            assertEquals(SharedMeshSource.INDIVIDUAL_SEARCH,
                    cold.source(),
                    "preferring the mesh cannot conjure an uncertified plan");
            assertTrue(cold.plan().join().usable());
            mesh.plan(meshRequest(request, "seed-2", 1,
                    NavigationStrategy.PREFER_SHARED)).plan().join();

            NavigationRequest moved = NavigationRequest.builder(instance,
                            new Pos(0.5000001, 40, 0.5),
                            new Pos(12.5, 40, 0.5),
                            EntityType.ZOMBIE.boundingBox(),
                            BuiltinNavigationProfiles.forEntityType(
                                    EntityType.ZOMBIE))
                    .maxPathLength(32).build();
            SharedMeshHandle shifted = mesh.plan(meshRequest(
                    moved, "shifted", 2, NavigationStrategy.PREFER_SHARED));
            assertEquals(SharedMeshSource.PARITY_CERTIFICATION_SEARCH,
                    shifted.source(),
                    "a source the mesh holds no certificate for must search");
            assertTrue(shifted.plan().join().usable());
            assertEquals(0, mesh.status().sharedPlans());
            cold.close();
            shifted.close();
        }
    }

    @Test
    void disablingAtRuntimeStopsSharingAndReleasesEveryRegion(Env env) {
        Instance instance = flatInstance(env);
        NavigationRequest request = crossingRequest(instance);

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8)
                .sharedMesh(SharedMeshOptions.enabledWith(seeding())).build()) {
            SharedMeshNavigation mesh = navigation.sharedMesh();
            mesh.plan(meshRequest(request, "seed-1", 0,
                    NavigationStrategy.PREFER_SHARED)).plan().join();
            mesh.plan(meshRequest(request, "seed-2", 1,
                    NavigationStrategy.PREFER_SHARED)).plan().join();
            assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    mesh.plan(meshRequest(request, "shared", 2,
                            NavigationStrategy.PREFER_SHARED)).source());
            assertEquals(1, mesh.status().regions().promotedRegions());

            mesh.disable();

            assertFalse(mesh.enabled());
            assertEquals(0, mesh.status().regions().regions(),
                    "disabling must release every retained region");
            SharedMeshHandle switched = mesh.plan(meshRequest(
                    request, "shared", 3, NavigationStrategy.PREFER_SHARED));
            assertEquals(SharedMeshSource.UNSHARED_SEARCH,
                    switched.source(),
                    "the same call site must stop reaching the mesh");
            assertTrue(switched.plan().join().usable());

            mesh.enable();

            assertTrue(mesh.enabled());
            SharedMeshHandle restarted = mesh.plan(meshRequest(
                    request, "shared", 4, NavigationStrategy.PREFER_SHARED));
            assertEquals(SharedMeshSource.INDIVIDUAL_SEARCH,
                    restarted.source(),
                    "re-enabling starts cold, not from released regions");
            assertTrue(restarted.plan().join().usable());
            switched.close();
            restarted.close();
        }
    }

    @Test
    void meshApiReachesTheSamePlanAsTheRawCoordinator(Env env) {
        Instance instance = flatInstance(env);
        NavigationRequest request = crossingRequest(instance);

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8)
                .sharedMesh(SharedMeshOptions.enabledWith(seeding())).build();
             AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            NavigationPlan throughMesh =
                    AdaptiveMeshTestSupport.promoteAndReplayThroughMesh(
                            navigation.sharedMesh(), request);
            NavigationPlan throughCoordinator =
                    AdaptiveMeshTestSupport.promoteAndReplay(service, request);

            assertTrue(NavigationPlanParity.semanticallyEquivalent(
                            throughCoordinator, throughMesh),
                    () -> "facade=" + throughMesh.nodes()
                            + " coordinator=" + throughCoordinator.nodes());
            assertEquals(2, navigation.metricsSnapshot().searches().submitted(),
                    "the replayed request must not have searched");
        }
    }

    private static Instance flatInstance(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 1,
                (x, z) -> instance.loadChunk(x, z).join());
        return instance;
    }

    private static NavigationRequest crossingRequest(Instance instance) {
        return NavigationRequest.builder(instance,
                        new Pos(0.5, 40, 0.5), new Pos(12.5, 40, 0.5),
                        EntityType.ZOMBIE.boundingBox(),
                        BuiltinNavigationProfiles.forEntityType(
                                EntityType.ZOMBIE))
                .maxPathLength(32).build();
    }

    private static SharedMeshPolicy seeding() {
        return SharedMeshPolicy.builder()
                .promotionRequests(2).retentionRequests(1).build();
    }

    private static SharedMeshRequest meshRequest(
            NavigationRequest request, Object actorKey, long tick,
            NavigationStrategy strategy) {
        return SharedMeshRequest.builder(request)
                .actor(actorKey).target("player")
                .worldRevision(1).currentTick(tick)
                .strategy(strategy).build();
    }
}
