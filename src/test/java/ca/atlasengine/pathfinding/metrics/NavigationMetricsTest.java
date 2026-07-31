package ca.atlasengine.pathfinding.metrics;

import ca.atlasengine.pathfinding.NavigationState;
import ca.atlasengine.pathfinding.metrics.LatencyDistribution;import ca.atlasengine.pathfinding.metrics.NavigationMetrics;import ca.atlasengine.pathfinding.metrics.NavigationMetricsSnapshot;import ca.atlasengine.pathfinding.adaptive.SharedMeshSource;
import org.junit.jupiter.api.Test;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class NavigationMetricsTest {
    @Test
    void freshRecorderReportsAnEmptyButUsableSnapshot() {
        NavigationMetricsSnapshot snapshot = new NavigationMetrics().snapshot();

        assertEquals(0, snapshot.searches().submitted());
        assertEquals(0, snapshot.searches().terminated());
        assertEquals(0, snapshot.searches().inFlight());
        assertEquals(0, snapshot.latency().count());
        assertEquals(0, snapshot.latency().p99Micros());
        assertEquals(0, snapshot.adaptiveSources().total());
        assertEquals(0.0, snapshot.adaptiveSources().sharedHitRate());
        assertEquals(0.0, snapshot.controllerOutcomes().stallRate());
        assertEquals(0.0, snapshot.controllerOutcomes().travelStallRate());
        assertEquals(0, snapshot.stalePlans());
        assertEquals(0, snapshot.queue().parallelism());
        assertEquals(0, snapshot.queue().queueCapacity());
    }

    @Test
    void eachRecordingMovesOnlyItsOwnCounter() {
        NavigationMetrics metrics = new NavigationMetrics();
        metrics.searchSubmitted();
        metrics.searchSubmitted();
        metrics.searchSubmitted();
        metrics.searchSubmitted();
        metrics.searchSubmitted();
        metrics.searchCompleted(1_000_000);
        metrics.searchFailed();
        metrics.searchCancelled();
        metrics.searchRejected();
        metrics.searchSuperseded();

        NavigationMetricsSnapshot.SearchCounts counts =
                metrics.snapshot().searches();
        assertEquals(5, counts.submitted());
        assertEquals(1, counts.completed());
        assertEquals(1, counts.failed());
        assertEquals(1, counts.cancelled());
        assertEquals(1, counts.rejected());
        assertEquals(1, counts.superseded());
        assertEquals(5, counts.terminated());
        assertEquals(0, counts.inFlight());
    }

    @Test
    void snapshotKeepsSubmittedAtOrAboveTerminatedAndLatencyBelowCompleted() {
        NavigationMetrics metrics = new NavigationMetrics();
        for (int index = 0; index < 40; index++) {
            metrics.searchSubmitted();
            metrics.searchCompleted(250_000);
        }
        metrics.searchSubmitted();

        NavigationMetricsSnapshot snapshot = metrics.snapshot();
        assertTrue(snapshot.searches().submitted()
                >= snapshot.searches().terminated());
        assertTrue(snapshot.latency().count()
                <= snapshot.searches().completed());
        assertEquals(1, snapshot.searches().inFlight());
    }

    @Test
    void queueWaitAndTotalLatencyAreSeparateSeries() {
        NavigationMetrics metrics = new NavigationMetrics();
        metrics.searchStarted(2_000_000);
        metrics.searchStarted(2_000_000);
        metrics.searchCompleted(9_000_000);

        NavigationMetricsSnapshot snapshot = metrics.snapshot();
        assertEquals(2, snapshot.queueWait().count());
        assertEquals(1, snapshot.latency().count());
        assertEquals(2000, snapshot.queueWait().maximumMicros());
        assertEquals(9000, snapshot.latency().maximumMicros());
    }

    @Test
    void percentilesExposeATailThatTheMeanHides() {
        NavigationMetrics metrics = new NavigationMetrics();
        for (int index = 0; index < 90; index++) {
            metrics.searchCompleted(100_000);
        }
        for (int index = 0; index < 10; index++) {
            metrics.searchCompleted(3_000_000);
        }

        LatencyDistribution latency = metrics.snapshot().latency();
        assertEquals(100, latency.count());
        assertEquals(3000, latency.maximumMicros());
        assertEquals(390.0, latency.meanMicros(), 1.0e-9);
        assertTrue(latency.p50Micros() >= 100 && latency.p50Micros() < 1000,
                latency::toString);
        assertEquals(3000, latency.p99Micros());
        assertEquals(3000, latency.percentileMicros(1.0));
    }

    @Test
    void bucketsCoverEveryMagnitudeAndSaturateInsteadOfOverflowing() {
        assertEquals(0, LatencyDistribution.bucketOf(0));
        assertEquals(1, LatencyDistribution.bucketOf(1));
        assertEquals(2, LatencyDistribution.bucketOf(2));
        assertEquals(2, LatencyDistribution.bucketOf(3));
        assertEquals(3, LatencyDistribution.bucketOf(4));
        assertEquals(LatencyDistribution.BUCKETS - 1,
                LatencyDistribution.bucketOf(Long.MAX_VALUE));
        assertEquals(0, LatencyDistribution.bucketUpperBoundMicros(0));
        assertEquals(3, LatencyDistribution.bucketUpperBoundMicros(2));
        assertEquals(Long.MAX_VALUE, LatencyDistribution.bucketUpperBoundMicros(
                LatencyDistribution.BUCKETS - 1));
        assertThrows(IllegalArgumentException.class,
                () -> LatencyDistribution.empty().percentileMicros(1.5));
    }

    @Test
    void windowedDistributionDescribesOnlyTheSamplesSinceTheEarlierSnapshot() {
        NavigationMetrics metrics = new NavigationMetrics();
        for (int index = 0; index < 50; index++) metrics.searchCompleted(50_000);
        LatencyDistribution before = metrics.snapshot().latency();
        for (int index = 0; index < 10; index++) metrics.searchCompleted(4_000_000);

        LatencyDistribution window = metrics.snapshot().latency().since(before);
        assertEquals(10, window.count());
        assertEquals(4000.0, window.meanMicros(), 1.0);
        assertTrue(window.p50Micros() >= 4000, window::toString);
        assertEquals(0, window.since(window).count());
    }

    @Test
    void adaptiveSourcesAreCountedPerDispatchAndYieldTheMeshHitRate() {
        NavigationMetrics metrics = new NavigationMetrics();
        metrics.adaptiveDispatch(SharedMeshSource.INDIVIDUAL_SEARCH);
        metrics.adaptiveDispatch(SharedMeshSource.SHARED_TARGET_FIELD);
        metrics.adaptiveDispatch(SharedMeshSource.SHARED_TARGET_FIELD);
        metrics.adaptiveDispatch(
                SharedMeshSource.PARITY_CERTIFICATION_SEARCH);
        metrics.adaptiveDispatch(SharedMeshSource.UNTRACKED_FALLBACK);

        NavigationMetricsSnapshot.AdaptiveSourceCounts sources =
                metrics.snapshot().adaptiveSources();
        assertEquals(1, sources.individualSearch());
        assertEquals(2, sources.sharedTargetField());
        assertEquals(1, sources.parityCertificationSearch());
        assertEquals(1, sources.untrackedFallback());
        assertEquals(5, sources.total());
        assertEquals(0.4, sources.sharedHitRate(), 1.0e-9);
        assertEquals(0, metrics.snapshot().searches().submitted(),
                "an adaptive dispatch is not itself a search submission");
    }

    @Test
    void onlyTerminalControllerStatesBecomeOutcomes() {
        NavigationMetrics metrics = new NavigationMetrics();
        metrics.controllerOutcome(NavigationState.IDLE);
        metrics.controllerOutcome(NavigationState.COMPUTING);
        metrics.controllerOutcome(NavigationState.FOLLOWING);
        metrics.controllerOutcome(NavigationState.PARTIAL);
        metrics.controllerOutcome(null);
        metrics.controllerOutcome(NavigationState.COMPLETED);
        metrics.controllerOutcome(NavigationState.COMPLETED);
        metrics.controllerOutcome(NavigationState.COMPLETED);
        metrics.controllerOutcome(NavigationState.STUCK);
        metrics.controllerOutcome(NavigationState.FAILED);

        NavigationMetricsSnapshot.ControllerOutcomeCounts outcomes =
                metrics.snapshot().controllerOutcomes();
        assertEquals(3, outcomes.completed());
        assertEquals(1, outcomes.stuck());
        assertEquals(1, outcomes.failed());
        assertEquals(0, outcomes.cancelled());
        assertEquals(5, outcomes.total());
        assertEquals(0.2, outcomes.stallRate(), 1.0e-9);
    }

    @Test
    void queueGaugesAppearOnlyWhileBound() {
        NavigationMetrics metrics = new NavigationMetrics();
        metrics.bindQueueGauges(new NavigationMetrics.QueueGauges() {
            @Override
            public int activeSearches() {
                return 3;
            }

            @Override
            public int queuedSearches() {
                return 7;
            }

            @Override
            public int parallelism() {
                return 4;
            }

            @Override
            public int queueCapacity() {
                return 64;
            }
        });
        assertEquals(new NavigationMetricsSnapshot.QueueState(3, 7, 64, 4),
                metrics.snapshot().queue());

        metrics.bindQueueGauges(null);
        assertEquals(new NavigationMetricsSnapshot.QueueState(0, 0, 0, 0),
                metrics.snapshot().queue());
    }

    @Test
    void noOpCompletionsDiluteStallRateButNotTheTravelFilteredRate() {
        NavigationMetrics metrics = new NavigationMetrics();
        // One real stall against nine navigations that had ground to cover,
        // buried under ninety mobs that replanned while standing on the goal.
        metrics.controllerOutcome(NavigationState.STUCK, true);
        for (int index = 0; index < 9; index++) {
            metrics.controllerOutcome(NavigationState.COMPLETED, true);
        }
        for (int index = 0; index < 90; index++) {
            metrics.controllerOutcome(NavigationState.COMPLETED, false);
        }

        NavigationMetricsSnapshot.ControllerOutcomeCounts outcomes =
                metrics.snapshot().controllerOutcomes();
        assertEquals(100, outcomes.total());
        assertEquals(1, outcomes.stuck());
        assertEquals(10, outcomes.travelling());
        assertEquals(0.01, outcomes.stallRate(), 1.0e-9);
        assertEquals(0.10, outcomes.travelStallRate(), 1.0e-9);
        assertTrue(outcomes.travelStallRate() > outcomes.stallRate(),
                "a smaller denominator over the same stalls cannot report less");
    }

    @Test
    void aStallCountsAsTravellingHoweverItWasClassified() {
        NavigationMetrics metrics = new NavigationMetrics();
        metrics.controllerOutcome(NavigationState.STUCK, false);
        metrics.controllerOutcome(NavigationState.COMPLETED, false);

        NavigationMetricsSnapshot.ControllerOutcomeCounts outcomes =
                metrics.snapshot().controllerOutcomes();
        assertEquals(2, outcomes.total());
        assertEquals(1, outcomes.travelling());
        assertEquals(0.5, outcomes.stallRate(), 1.0e-9);
        assertEquals(1.0, outcomes.travelStallRate(), 1.0e-9);
    }

    @Test
    void aNavigationWithGroundToCoverIsTheDefaultForTheSingleArgumentRecording() {
        NavigationMetrics metrics = new NavigationMetrics();
        metrics.controllerOutcome(NavigationState.STUCK);
        metrics.controllerOutcome(NavigationState.COMPLETED);
        metrics.controllerOutcome(NavigationState.FOLLOWING, true);
        metrics.controllerOutcome(null, true);

        NavigationMetricsSnapshot.ControllerOutcomeCounts outcomes =
                metrics.snapshot().controllerOutcomes();
        assertEquals(2, outcomes.total());
        assertEquals(2, outcomes.travelling(),
                "an unclassified outcome is assumed to have had to move");
        assertEquals(outcomes.stallRate(), outcomes.travelStallRate(), 1.0e-9);
    }

    @Test
    void aPlanRefusedForStalenessIsCountedWithoutTouchingSearchCounters() {
        NavigationMetrics metrics = new NavigationMetrics();
        metrics.searchSubmitted();
        metrics.searchCompleted(1_000_000);
        metrics.planTooStaleToFollow();
        metrics.planTooStaleToFollow();

        NavigationMetricsSnapshot snapshot = metrics.snapshot();
        assertEquals(2, snapshot.stalePlans());
        assertEquals(1, snapshot.searches().completed());
        assertEquals(0, snapshot.searches().failed());
        assertEquals(0, snapshot.controllerOutcomes().total(),
                "a refused hand-off is not a finished navigation");
    }

    @Test
    void concurrentRecordingLosesNothingAndNeverReportsANegativeRate()
            throws Exception {
        NavigationMetrics metrics = new NavigationMetrics();
        int threads = 8;
        int perThread = 20_000;
        ExecutorService pool = Executors.newFixedThreadPool(threads);
        CountDownLatch start = new CountDownLatch(1);
        NavigationMetricsSnapshot first;
        try {
            for (int worker = 0; worker < threads; worker++) {
                pool.execute(() -> {
                    try {
                        start.await();
                    } catch (InterruptedException interrupted) {
                        Thread.currentThread().interrupt();
                        return;
                    }
                    for (int index = 0; index < perThread; index++) {
                        metrics.searchSubmitted();
                        metrics.searchStarted(1_000);
                        metrics.searchCompleted(index * 1_000L);
                        metrics.adaptiveDispatch(
                                SharedMeshSource.SHARED_TARGET_FIELD);
                        metrics.controllerOutcome(NavigationState.COMPLETED);
                    }
                });
            }
            first = metrics.snapshot();
            start.countDown();
            pool.shutdown();
            assertTrue(pool.awaitTermination(60, TimeUnit.SECONDS));
        } finally {
            pool.shutdownNow();
        }

        NavigationMetricsSnapshot last = metrics.snapshot();
        long expected = (long) threads * perThread;
        assertEquals(expected, last.searches().submitted());
        assertEquals(expected, last.searches().completed());
        assertEquals(expected, last.latency().count());
        assertEquals(expected, last.adaptiveSources().sharedTargetField());
        assertEquals(expected, last.controllerOutcomes().completed());
        assertTrue(last.searches().submitted()
                >= first.searches().submitted());
        assertFalse(last.elapsedSecondsSince(first) < 0);
    }
}
