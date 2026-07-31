package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.metrics.NavigationMetrics;import ca.atlasengine.pathfinding.metrics.NavigationMetricsSnapshot;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.util.ArrayList;
import java.util.concurrent.CompletionException;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.*;

class AsyncEntityPathfindingServiceTest {
    private static final BoundingBox BOX = new BoundingBox(0.8, 1.8, 0.8);
    private static final NavigationProfile GROUND = NavigationProfile.builder(NavigationMode.GROUND, MobTraversalProfile.DEFAULT, GroundCapabilities.STANDARD).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();

    @Test
    void runsAllRequestsOnBoundedNamedWorkers() {
        RecordingWorld world = new RecordingWorld();
        try (var service = new AsyncEntityPathfindingService(4, 64)) {
            var futures = new ArrayList<java.util.concurrent.CompletableFuture<PathResult>>();
            for (int index = 0; index < 32; index++) {
                futures.add(service.submit(request(world, index % 9 - 4)));
            }
            futures.forEach(future -> assertTrue(future.join().found()));

            assertTrue(world.workerObserved);
            assertEquals(4, service.parallelism());
            assertEquals(0, service.queuedCount());
        }
    }

    @Test
    void overloadIsRejectedInsteadOfGrowingTheQueue() throws Exception {
        BlockingWorld world = new BlockingWorld();
        try (var service = new AsyncEntityPathfindingService(1, 1)) {
            var running = service.submit(request(world, 0));
            assertTrue(world.entered.await(2, TimeUnit.SECONDS));
            var queued = service.submit(request(world, 1));
            var rejected = service.submit(request(world, 2));

            PathResult shed = rejected.join();
            assertEquals(PathStatus.SHED, shed.status());
            assertTrue(shed.shed());
            assertFalse(shed.found());
            assertTrue(shed.nodes().isEmpty());
            assertSame(PathResult.SHED, shed,
                    "shedding must not allocate a result per rejection");
            assertEquals(1, service.queuedCount());
            assertEquals(1, service.queueCapacity());

            world.release.countDown();
            assertTrue(running.join().found());
            assertTrue(queued.join().found());
        }
    }

    @Test
    void aShedSearchCompletesWithAValueRatherThanAnException() throws Exception {
        BlockingWorld world = new BlockingWorld();
        NavigationMetrics metrics = new NavigationMetrics();
        try (var service = new AsyncEntityPathfindingService(1, 1, metrics)) {
            var running = service.submit(request(world, 0));
            assertTrue(world.entered.await(2, TimeUnit.SECONDS));
            service.submit(request(world, 1));
            var shed = service.submit(request(world, 2));

            assertFalse(shed.isCompletedExceptionally(),
                    "routine backpressure must not look like a defect");
            assertFalse(shed.isCancelled());
            assertTrue(shed.isDone());
            assertEquals(PathStatus.SHED, shed.getNow(null).status());

            NavigationMetricsSnapshot snapshot = metrics.snapshot();
            assertEquals(1, snapshot.searches().rejected());
            assertEquals(0, snapshot.searches().failed());
            assertEquals(1, snapshot.queue().queueCapacity());

            world.release.countDown();
            assertTrue(running.join().found());
        }
    }

    @Test
    void cancellationReachesAnActiveSearch() throws Exception {
        BlockingWorld world = new BlockingWorld();
        try (var service = new AsyncEntityPathfindingService(1, 2)) {
            var future = service.submit(request(world, 0));
            assertTrue(world.entered.await(2, TimeUnit.SECONDS));

            assertTrue(future.cancel(true));
            world.release.countDown();

            assertThrows(java.util.concurrent.CancellationException.class, future::join);
            await(() -> service.activeCount() == 0, Duration.ofSeconds(2));
        }
    }

    @Test
    void latestRequestCancelsRunningStaleResult() throws Exception {
        BlockingWorld world = new BlockingWorld();
        try (var service = new AsyncEntityPathfindingService(1, 2)) {
            var stale = service.submitLatest("mob", request(world, 0));
            assertTrue(world.entered.await(2, TimeUnit.SECONDS));
            var latest = service.submitLatest("mob", request(world, 3));

            assertTrue(stale.isCancelled());
            world.release.countDown();
            PathResult result = latest.join();
            assertTrue(result.found());
            assertEquals(3.5, result.nodes().get(result.nodes().size() - 1).z(), 0.01);
        }
    }

    @Test
    void latestRequestEvictsQueuedStaleWorkBeforeEnqueueingReplacement() throws Exception {
        BlockingWorld world = new BlockingWorld();
        try (var service = new AsyncEntityPathfindingService(1, 1)) {
            var running = service.submit(request(world, -3));
            assertTrue(world.entered.await(2, TimeUnit.SECONDS));
            var staleQueued = service.submitLatest("mob", request(world, 0));

            // This must fit despite queue capacity one because staleQueued is removed.
            var latest = service.submitLatest("mob", request(world, 3));
            assertTrue(staleQueued.isCancelled());
            assertEquals(1, service.queuedCount());

            world.release.countDown();
            assertTrue(running.join().found());
            assertTrue(latest.join().found());
        }
    }

    @Test
    void concurrentReplansPublishOnlyTheNewestFuture() throws Exception {
        BlockingWorld world = new BlockingWorld();
        try (var service = new AsyncEntityPathfindingService(1, 16)) {
            var initial = service.submitLatest("mob", request(world, -4));
            assertTrue(world.entered.await(2, TimeUnit.SECONDS));

            var submitted = new ArrayList<java.util.concurrent.CompletableFuture<PathResult>>();
            for (int z = -3; z <= 4; z++) {
                submitted.add(service.submitLatest("mob", request(world, z)));
            }
            for (int index = 0; index < submitted.size() - 1; index++) {
                assertTrue(submitted.get(index).isCancelled());
            }
            assertTrue(initial.isCancelled());

            world.release.countDown();
            var newest = submitted.get(submitted.size() - 1).join();
            assertTrue(newest.found());
            assertEquals(4.5, newest.nodes().get(newest.nodes().size() - 1).z(), 0.01);
        }
    }

    @Test
    void closeCancelsRunningAndQueuedFutures() throws Exception {
        BlockingWorld world = new BlockingWorld();
        var service = new AsyncEntityPathfindingService(1, 2);
        var running = service.submit(request(world, 0));
        assertTrue(world.entered.await(2, TimeUnit.SECONDS));
        var queued = service.submit(request(world, 1));

        service.close();

        assertTrue(running.isCancelled());
        assertTrue(queued.isCancelled());
    }

    @Test
    void everySearchReachesExactlyOneTerminalCounter() throws Exception {
        BlockingWorld world = new BlockingWorld();
        NavigationMetrics metrics = new NavigationMetrics();
        try (var service = new AsyncEntityPathfindingService(1, 1, metrics)) {
            var running = service.submit(request(world, 0));
            assertTrue(world.entered.await(2, TimeUnit.SECONDS));
            var superseded = service.submitLatest("mob", request(world, 1));
            var shed = service.submit(request(world, 2));
            assertTrue(shed.join().shed());
            var replacement = service.submitLatest("mob", request(world, 3));

            world.release.countDown();
            assertTrue(running.join().found());
            assertTrue(replacement.join().found());
            assertTrue(superseded.isCancelled());

            NavigationMetricsSnapshot snapshot = metrics.snapshot();
            assertEquals(4, snapshot.searches().submitted());
            assertEquals(2, snapshot.searches().completed());
            assertEquals(1, snapshot.searches().rejected());
            assertEquals(1, snapshot.searches().superseded());
            assertEquals(0, snapshot.searches().cancelled());
            assertEquals(0, snapshot.searches().failed());
            assertEquals(4, snapshot.searches().terminated());
            assertEquals(0, snapshot.searches().inFlight());
        }
    }

    /**
     * The counters above are assertable without a poll only because a search
     * records its terminal counter before it publishes its terminal future
     * state. Each future below reads the recorder from its own completion, so
     * a counter that trailed the completion fails here deterministically
     * instead of once per few hundred runs under load.
     */
    @Test
    void terminalCountersAreVisibleTheInstantAFutureCompletes()
            throws Exception {
        BlockingWorld world = new BlockingWorld();
        NavigationMetrics metrics = new NavigationMetrics();
        try (var service = new AsyncEntityPathfindingService(1, 2, metrics)) {
            var running = service.submit(request(world, 0));
            assertTrue(world.entered.await(2, TimeUnit.SECONDS));
            var supersededSeen = service.submitLatest("mob", request(world, 1))
                    .handle((ignored, failure) ->
                            metrics.snapshot().searches().superseded());
            var replacement = service.submitLatest("mob", request(world, 2));
            assertEquals(1, supersededSeen.join());

            var runningSeen = running.thenApply(ignored ->
                    metrics.snapshot().searches().completed());
            var replacementSeen = replacement.thenApply(ignored ->
                    metrics.snapshot().searches().completed());
            world.release.countDown();
            assertEquals(1, runningSeen.join());
            assertEquals(2, replacementSeen.join());

            BlockingWorld second = new BlockingWorld();
            var doomed = service.submit(request(second, 3));
            var cancelledSeen = doomed.handle((ignored, failure) ->
                    metrics.snapshot().searches().cancelled());
            assertTrue(second.entered.await(2, TimeUnit.SECONDS));
            assertTrue(doomed.cancel(true));
            assertEquals(1, cancelledSeen.join());
            second.release.countDown();

            var exploded = service.submit(request(
                    (x, y, z, condition) -> {
                        throw new IllegalStateException("world read failed");
                    }, 4));
            var failedSeen = exploded.handle((ignored, failure) ->
                    metrics.snapshot().searches().failed());
            assertEquals(1, failedSeen.join());
        }
    }

    @Test
    void latencyAndQueueWaitAreRecordedOnlyForSearchesThatRan() {
        RecordingWorld world = new RecordingWorld();
        NavigationMetrics metrics = new NavigationMetrics();
        try (var service = new AsyncEntityPathfindingService(2, 64, metrics)) {
            var futures = new ArrayList<java.util.concurrent.CompletableFuture<PathResult>>();
            for (int index = 0; index < 16; index++) {
                futures.add(service.submit(request(world, index % 5 - 2)));
            }
            futures.forEach(future -> assertTrue(future.join().found()));

            NavigationMetricsSnapshot snapshot = metrics.snapshot();
            assertEquals(16, snapshot.latency().count());
            assertEquals(16, snapshot.queueWait().count());
            assertTrue(snapshot.latency().maximumMicros() > 0);
            assertTrue(snapshot.latency().p99Micros()
                    >= snapshot.latency().p50Micros());
            assertTrue(snapshot.latency().meanMicros() > 0);
            assertEquals(2, snapshot.queue().parallelism());
            assertEquals(0, snapshot.adaptiveSources().total(),
                    "a plain service performs no adaptive dispatch");
        }
    }

    @Test
    void aSearchCancelledWhileQueuedNeverRecordsLatency() throws Exception {
        BlockingWorld world = new BlockingWorld();
        NavigationMetrics metrics = new NavigationMetrics();
        try (var service = new AsyncEntityPathfindingService(1, 4, metrics)) {
            var running = service.submit(request(world, 0));
            assertTrue(world.entered.await(2, TimeUnit.SECONDS));
            var queued = service.submit(request(world, 1));
            assertTrue(queued.cancel(true));

            world.release.countDown();
            assertTrue(running.join().found());
            await(() -> service.activeCount() == 0, Duration.ofSeconds(2));

            NavigationMetricsSnapshot snapshot = metrics.snapshot();
            assertEquals(2, snapshot.searches().submitted());
            assertEquals(1, snapshot.searches().completed());
            assertEquals(1, snapshot.searches().cancelled());
            assertEquals(1, snapshot.latency().count());
            assertEquals(1, snapshot.queueWait().count());
        }
    }

    @Test
    void failuresAreCountedSeparatelyFromCancellations() {
        NavigationMetrics metrics = new NavigationMetrics();
        Block.Getter exploding = (x, y, z, condition) -> {
            throw new IllegalStateException("world read failed");
        };
        try (var service = new AsyncEntityPathfindingService(1, 4, metrics)) {
            assertThrows(CompletionException.class,
                    () -> service.submit(request(exploding, 0)).join());

            NavigationMetricsSnapshot snapshot = metrics.snapshot();
            assertEquals(1, snapshot.searches().submitted());
            assertEquals(1, snapshot.searches().failed());
            assertEquals(0, snapshot.searches().completed());
            assertEquals(0, snapshot.searches().cancelled());
            assertEquals(0, snapshot.latency().count());
        }
    }

    private static NavigationRequest request(Block.Getter world, int targetZ) {
        return NavigationRequest.builder(
                        world,
                        new Pos(-8.5, 1, 0.5),
                        new Pos(8.5, 1, targetZ + 0.5),
                        BOX,
                        GROUND)
                .maxPathLength(100)
                .maxVisitedMultiplier(4)
                .build();
    }

    private static TestWorld flat() {
        return new TestWorld().floor(-40, 40, -12, 12, 0, Block.STONE);
    }

    private static void await(Check check, Duration timeout) throws Exception {
        long deadline = System.nanoTime() + timeout.toNanos();
        while (!check.value() && System.nanoTime() < deadline) {
            Thread.sleep(5);
        }
        assertTrue(check.value());
    }

    @FunctionalInterface
    private interface Check {
        boolean value();
    }

    private static final class RecordingWorld implements Block.Getter {
        private final TestWorld delegate = flat();
        private volatile boolean workerObserved;

        @Override
        public Block getBlock(int x, int y, int z, Condition condition) {
            workerObserved |= Thread.currentThread().getName()
                    .startsWith("minestom-entity-pathfinding-");
            return delegate.getBlock(x, y, z, condition);
        }
    }

    private static final class BlockingWorld implements Block.Getter {
        private final TestWorld delegate = flat();
        private final CountDownLatch entered = new CountDownLatch(1);
        private final CountDownLatch release = new CountDownLatch(1);

        @Override
        public Block getBlock(int x, int y, int z, Condition condition) {
            entered.countDown();
            try {
                release.await();
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
            }
            return delegate.getBlock(x, y, z, condition);
        }
    }
}
