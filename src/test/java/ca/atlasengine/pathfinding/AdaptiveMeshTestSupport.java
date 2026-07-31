package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.adaptive.SharedMeshPolicy;
import ca.atlasengine.pathfinding.internal.adaptive.SharedMeshCoordinator;
import ca.atlasengine.pathfinding.adaptive.SharedMeshHandle;
import ca.atlasengine.pathfinding.adaptive.SharedMeshSource;
import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.internal.adaptive.NavigationPlanParity;
import ca.atlasengine.pathfinding.adaptive.SharedMeshNavigation;
import ca.atlasengine.pathfinding.adaptive.SharedMeshRequest;

import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class AdaptiveMeshTestSupport {
    private AdaptiveMeshTestSupport() {
    }

    static NavigationPlan promoteAndReplay(
            AsyncEntityPathfindingService service,
            NavigationRequest request) {
        AtomicInteger searches = new AtomicInteger();
        try (SharedMeshCoordinator coordinator =
                     new SharedMeshCoordinator((actor, submitted) -> {
                         searches.incrementAndGet();
                         return service.submit(submitted).thenApply(result ->
                                 NavigationPlan.from(submitted, result));
                     }, SharedMeshPolicy.builder()
                             .promotionRequests(2)
                             .retentionRequests(1)
                             .build())) {
            NavigationPlan first = coordinator.request(
                    "seed-1", "target", 1, 0, request).plan().join();
            NavigationPlan second = coordinator.request(
                    "seed-2", "target", 1, 1, request).plan().join();
            assertTrue(NavigationPlanParity.semanticallyEquivalent(
                    first, second));
            SharedMeshHandle shared = coordinator.request(
                    "shared", "target", 1, 2, request);
            assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    shared.source());
            NavigationPlan replay = shared.plan().join();
            assertTrue(NavigationPlanParity.semanticallyEquivalent(
                    first, replay),
                    () -> "individual=" + first.nodes()
                            + " shared=" + replay.nodes());
            assertEquals(2, searches.get());
            return replay;
        }
    }

    /** The same sequence, driven through the public shared-mesh API. */
    static NavigationPlan promoteAndReplayThroughMesh(
            SharedMeshNavigation mesh, NavigationRequest request) {
        NavigationPlan first = mesh.plan(
                seed(request, "seed-1", 0)).plan().join();
        NavigationPlan second = mesh.plan(
                seed(request, "seed-2", 1)).plan().join();
        assertTrue(NavigationPlanParity.semanticallyEquivalent(first, second));
        SharedMeshHandle shared = mesh.plan(
                seed(request, "shared", 2));
        assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                shared.source());
        NavigationPlan replay = shared.plan().join();
        assertTrue(NavigationPlanParity.semanticallyEquivalent(first, replay),
                () -> "individual=" + first.nodes()
                        + " shared=" + replay.nodes());
        return replay;
    }

    private static SharedMeshRequest seed(
            NavigationRequest request, Object actorKey, long tick) {
        return SharedMeshRequest.builder(request)
                .actor(actorKey).target("target")
                .worldRevision(1).currentTick(tick).build();
    }
}
