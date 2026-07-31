package ca.atlasengine.pathfinding.metrics;

import ca.atlasengine.pathfinding.NavigationState;import ca.atlasengine.pathfinding.adaptive.SharedMeshSource;

import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.LongAdder;

/**
 * Cumulative, lock-free navigation telemetry shared by one search service, its
 * adaptive coordinator, and the controllers they drive.
 *
 * <p>Every recording method is a striped {@link LongAdder} increment plus, for
 * latency, one bucket increment and one relaxed maximum update. Nothing locks,
 * blocks, or allocates, so worker threads may record freely on the hot path.
 * Counters are monotonic and never reset; operators difference two
 * {@link #snapshot()} readings to obtain rates.</p>
 *
 * <p>Instances are safe for unrestricted concurrent use.</p>
 */
public final class NavigationMetrics {
    /** Instantaneous worker-pool state, published by the owning service. */
    public interface QueueGauges {
        int activeSearches();

        int queuedSearches();

        int parallelism();

        /**
         * The configured bound on {@link #queuedSearches()}, so a dashboard
         * can chart queue depth against the value a builder was given.
         */
        int queueCapacity();
    }

    private static final QueueGauges UNBOUND = new QueueGauges() {
        @Override
        public int activeSearches() {
            return 0;
        }

        @Override
        public int queuedSearches() {
            return 0;
        }

        @Override
        public int parallelism() {
            return 0;
        }

        @Override
        public int queueCapacity() {
            return 0;
        }
    };

    private static final SharedMeshSource[] SOURCES =
            SharedMeshSource.values();

    private final LongAdder submitted = new LongAdder();
    private final LongAdder completed = new LongAdder();
    private final LongAdder failed = new LongAdder();
    private final LongAdder cancelled = new LongAdder();
    private final LongAdder rejected = new LongAdder();
    private final LongAdder superseded = new LongAdder();
    private final LongAdder controllerCompleted = new LongAdder();
    private final LongAdder controllerStuck = new LongAdder();
    private final LongAdder controllerFailed = new LongAdder();
    private final LongAdder controllerCancelled = new LongAdder();
    private final LongAdder controllerTravelling = new LongAdder();
    private final LongAdder controllerUnreachable = new LongAdder();
    private final LongAdder stalePlans = new LongAdder();
    private final LongAdder[] adaptiveSources = new LongAdder[SOURCES.length];
    private final Histogram latency = new Histogram();
    private final Histogram queueWait = new Histogram();
    private volatile QueueGauges gauges = UNBOUND;

    public NavigationMetrics() {
        for (int index = 0; index < adaptiveSources.length; index++) {
            adaptiveSources[index] = new LongAdder();
        }
    }

    /**
     * Publishes worker-pool gauges into {@link #snapshot()}. When several
     * services share one recorder the most recent binding wins.
     */
    public void bindQueueGauges(QueueGauges bound) {
        gauges = bound == null ? UNBOUND : bound;
    }

    public void searchSubmitted() {
        submitted.increment();
    }

    /** Records the wait between submission and the start of execution. */
    public void searchStarted(long queueWaitNanos) {
        queueWait.record(queueWaitNanos);
    }

    /** Records a search that produced a result, end to end from submission. */
    public void searchCompleted(long totalNanos) {
        latency.record(totalNanos);
        completed.increment();
    }

    public void searchFailed() {
        failed.increment();
    }

    public void searchCancelled() {
        cancelled.increment();
    }

    /** Records a search shed because the bounded queue was full. */
    public void searchRejected() {
        rejected.increment();
    }

    /**
     * Records a plan refused by a follower because the entity moved beyond
     * the start tolerance while the search ran. The caller replans; nothing
     * about the search itself failed.
     */
    public void planTooStaleToFollow() {
        stalePlans.increment();
    }

    /** Records a search cancelled because a newer one replaced its key. */
    public void searchSuperseded() {
        superseded.increment();
    }

    public void adaptiveDispatch(SharedMeshSource source) {
        if (source != null) adaptiveSources[source.ordinal()].increment();
    }

    /**
     * Records a navigation the planner stopped chaining segments for, once per
     * navigation, at the moment that is decided.
     *
     * <p>This is not a stall: the follower is healthy and standing on a route
     * it finished, and the target is the thing that cannot be reached. It is
     * therefore kept out of {@code stuck} and out of the
     * {@link NavigationMetricsSnapshot.ControllerOutcomeCounts#travelStallRate()}
     * numerator, which before this counter existed absorbed every one of them
     * two hundred ticks late by way of the stall watchdog.</p>
     */
    public void controllerTargetUnreachable() {
        controllerUnreachable.increment();
    }

    /** Records a terminal controller state; other states are ignored. */
    public void controllerOutcome(NavigationState outcome) {
        controllerOutcome(outcome, true);
    }

    /**
     * Records a terminal controller state together with whether that
     * navigation had ground to cover.
     *
     * <p>A navigation that starts inside its own arrival tolerance completes
     * on its first tick without moving. Counting those in the stall
     * denominator dilutes it without bound, so they are excluded from
     * {@link NavigationMetricsSnapshot.ControllerOutcomeCounts#travelStallRate()}
     * while still appearing in every raw outcome counter. A stall is by
     * definition a navigation that had somewhere to go, so {@code STUCK}
     * counts as travelling whatever the caller passes.</p>
     */
    public void controllerOutcome(
            NavigationState outcome, boolean travelRequired) {
        if (outcome == null) return;
        switch (outcome) {
            case COMPLETED -> controllerCompleted.increment();
            case STUCK -> controllerStuck.increment();
            case FAILED -> controllerFailed.increment();
            case CANCELLED -> controllerCancelled.increment();
            default -> {
                return;
            }
        }
        if (travelRequired || outcome == NavigationState.STUCK) {
            controllerTravelling.increment();
        }
    }

    /**
     * Reads every counter once, terminal counters before {@code submitted} and
     * latency before its counter.
     *
     * <p>The result is a per-counter atomic, monotonically non-decreasing read
     * taken over a short interval rather than one global instant. Differencing
     * two snapshots therefore never yields a negative rate, and
     * {@code submitted >= terminated()} and
     * {@code latency.count() <= completed} hold in every snapshot; the residual
     * is the work genuinely in flight plus at most the handful of searches that
     * terminate while the snapshot is assembled. The three
     * {@link NavigationMetricsSnapshot.QueueState} fields read the executor's
     * own lock; every other field
     * is lock-free.</p>
     */
    public NavigationMetricsSnapshot snapshot() {
        LatencyDistribution latencySnapshot = latency.snapshot();
        LatencyDistribution queueWaitSnapshot = queueWait.snapshot();
        long completedCount = completed.sum();
        long failedCount = failed.sum();
        long cancelledCount = cancelled.sum();
        long rejectedCount = rejected.sum();
        long supersededCount = superseded.sum();
        QueueGauges bound = gauges;
        return new NavigationMetricsSnapshot(
                System.nanoTime(),
                new NavigationMetricsSnapshot.SearchCounts(
                        submitted.sum(), completedCount, failedCount,
                        cancelledCount, rejectedCount, supersededCount),
                latencySnapshot,
                queueWaitSnapshot,
                new NavigationMetricsSnapshot.AdaptiveSourceCounts(
                        adaptiveSources[
                                SharedMeshSource.INDIVIDUAL_SEARCH
                                        .ordinal()].sum(),
                        adaptiveSources[
                                SharedMeshSource.SHARED_TARGET_FIELD
                                        .ordinal()].sum(),
                        adaptiveSources[SharedMeshSource
                                .PARITY_CERTIFICATION_SEARCH.ordinal()].sum(),
                        adaptiveSources[
                                SharedMeshSource.UNTRACKED_FALLBACK
                                        .ordinal()].sum(),
                        adaptiveSources[
                                SharedMeshSource.UNSHARED_SEARCH
                                        .ordinal()].sum()),
                new NavigationMetricsSnapshot.ControllerOutcomeCounts(
                        controllerCompleted.sum(), controllerStuck.sum(),
                        controllerFailed.sum(), controllerCancelled.sum(),
                        controllerTravelling.sum(),
                        controllerUnreachable.sum()),
                stalePlans.sum(),
                new NavigationMetricsSnapshot.QueueState(
                        bound.activeSearches(), bound.queuedSearches(),
                        bound.queueCapacity(), bound.parallelism()));
    }

    private static final class Histogram {
        private final LongAdder[] buckets =
                new LongAdder[LatencyDistribution.BUCKETS];
        private final LongAdder totalMicros = new LongAdder();
        private final AtomicLong maximumMicros = new AtomicLong();

        Histogram() {
            for (int index = 0; index < buckets.length; index++) {
                buckets[index] = new LongAdder();
            }
        }

        void record(long nanos) {
            long micros = nanos <= 0 ? 0 : nanos / 1_000L;
            buckets[LatencyDistribution.bucketOf(micros)].increment();
            totalMicros.add(micros);
            // Contended only while a new worst case is being established.
            if (micros > maximumMicros.get()) {
                maximumMicros.accumulateAndGet(micros, Math::max);
            }
        }

        LatencyDistribution snapshot() {
            long[] counts = new long[buckets.length];
            for (int index = 0; index < buckets.length; index++) {
                counts[index] = buckets[index].sum();
            }
            return LatencyDistribution.of(
                    counts, totalMicros.sum(), maximumMicros.get());
        }
    }
}
