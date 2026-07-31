package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.NavigationRequest;import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ControllerPathSchedulerTest {
    @Test
    void tenThousandActorsRemainDeferredOutsideTheExecutorQueue() {
        CountDownLatch release = new CountDownLatch(1);
        NavigationRequest request = request(blockingWorld(release));
        try (var service = new AsyncEntityPathfindingService(1, 1);
             var scheduler = new ControllerPathScheduler(service,
                     new ControllerSchedulingOptions(10_000, 1, 60, 20))) {
            List<CompletableFuture<PathResult>> futures = new ArrayList<>(10_000);
            for (int actor = 0; actor < 10_000; actor++) {
                futures.add(scheduler.submitLatest(actor, request, 10));
            }

            assertEquals(1, scheduler.runningCount());
            assertEquals(9_999, scheduler.deferredCount());
            assertEquals(0, service.queuedCount(),
                    "controller population must not become executor backlog");
            futures.forEach(future -> future.cancel(true));
            release.countDown();
        }
    }

    @Test
    void urgentActorEvictsLowestPriorityDeferredWork() {
        CountDownLatch release = new CountDownLatch(1);
        NavigationRequest request = request(blockingWorld(release));
        try (var service = new AsyncEntityPathfindingService(1, 1);
             var scheduler = new ControllerPathScheduler(service,
                     new ControllerSchedulingOptions(2, 1, 60, 20))) {
            CompletableFuture<PathResult> running =
                    scheduler.submitLatest("running", request, 10);
            CompletableFuture<PathResult> lowOne =
                    scheduler.submitLatest("low-1", request, 1);
            CompletableFuture<PathResult> lowTwo =
                    scheduler.submitLatest("low-2", request, 1);
            CompletableFuture<PathResult> urgent =
                    scheduler.submitLatest("urgent", request, 100);

            assertTrue(lowTwo.isDone());
            assertSame(PathResult.SHED, lowTwo.join());
            assertEquals(2, scheduler.deferredCount());
            running.cancel(true);
            lowOne.cancel(true);
            urgent.cancel(true);
            release.countDown();
        }
    }

    @Test
    void latestIntentCancelsOlderDeferredIntentForActor() {
        CountDownLatch release = new CountDownLatch(1);
        NavigationRequest request = request(blockingWorld(release));
        try (var service = new AsyncEntityPathfindingService(1, 1);
             var scheduler = new ControllerPathScheduler(service,
                     new ControllerSchedulingOptions(8, 1, 60, 20))) {
            CompletableFuture<PathResult> blocker =
                    scheduler.submitLatest("blocker", request, 10);
            CompletableFuture<PathResult> old =
                    scheduler.submitLatest("actor", request, 10);
            CompletableFuture<PathResult> replacement =
                    scheduler.submitLatest("actor", request, 10);

            assertTrue(old.isCancelled());
            assertEquals(1, scheduler.deferredCount());
            blocker.cancel(true);
            replacement.cancel(true);
            release.countDown();
        }
    }

    @Test
    void completionCallbacksCannotInvertAnExternalCoordinatorLock() {
        Object coordinatorLock = new Object();
        CountDownLatch release = new CountDownLatch(1);
        CountDownLatch callbackStarted = new CountDownLatch(1);
        NavigationRequest request = request(blockingWorld(release));
        try (var service = new AsyncEntityPathfindingService(1, 1);
             var scheduler = new ControllerPathScheduler(service,
                     new ControllerSchedulingOptions(8, 1, 60, 20))) {
            CompletableFuture<PathResult> first =
                    scheduler.submitLatest("first", request, 10);
            first.whenComplete((ignored, failure) -> {
                callbackStarted.countDown();
                synchronized (coordinatorLock) {
                    // Models adaptive coordinator completion bookkeeping.
                }
            });

            CompletableFuture<Void> competingSubmission =
                    CompletableFuture.runAsync(() -> {
                        synchronized (coordinatorLock) {
                            release.countDown();
                            try {
                                assertTrue(callbackStarted.await(
                                        5, java.util.concurrent.TimeUnit.SECONDS));
                            } catch (InterruptedException interrupted) {
                                Thread.currentThread().interrupt();
                                throw new AssertionError(interrupted);
                            }
                            scheduler.submitLatest(
                                    "second", request(new TestWorld()), 10);
                        }
                    });
            assertTimeoutPreemptively(Duration.ofSeconds(5),
                    competingSubmission::join);
        }
    }

    private static NavigationRequest request(Block.Getter blocks) {
        return NavigationRequest.builder(blocks,
                        new Pos(0.5, 1, 0.5), new Pos(8.5, 1, 0.5),
                        new BoundingBox(0.6, 1.8, 0.6),
                        BuiltinNavigationProfiles.forEntityType(
                                EntityType.ZOMBIE))
                .maxPathLength(16).build();
    }

    private static Block.Getter blockingWorld(CountDownLatch release) {
        TestWorld world = new TestWorld().floor(-2, 10, -2, 2, 0, Block.STONE);
        return (x, y, z, condition) -> {
            assertTimeoutPreemptively(Duration.ofSeconds(5), () -> release.await());
            return world.getBlock(x, y, z, condition);
        };
    }
}
