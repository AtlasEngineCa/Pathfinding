package ca.atlasengine.pathfinding.load;

import ca.atlasengine.pathfinding.metrics.LatencyDistribution;
import ca.atlasengine.pathfinding.metrics.NavigationMetricsSnapshot;
import ca.atlasengine.pathfinding.NavigationState;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.LongAdder;

/**
 * Everything the load run observes that {@link NavigationMetricsSnapshot} does
 * not already publish: coordinator/thread/cache samples, tick-thread wall
 * times, follower-side plan disposition, and the exception taxonomy of the
 * handle futures the harness owns.
 */
final class LoadMetrics {
    record Sample(
            long tick, int regions, int promotedRegions, int coordinatorActors,
            int targets, int retainedNodes, int observedPlans,
            int certifiedPlans, int publishedPlans, int planCache, int queued,
            int active, int parallelism, int queueCapacity, int threads,
            int pathfindingThreads, long submitted, long terminated,
            long latencyCount, long adaptiveDispatches) {

        int retainedPlans() {
            return observedPlans + certifiedPlans + publishedPlans;
        }
    }

    private final Map<NavigationState, LongAdder> travelTerminals =
            new EnumMap<>(NavigationState.class);
    private final LongAdder travelEpisodes = new LongAdder();
    private final LongAdder adaptiveCalls = new LongAdder();
    private final LongAdder unusablePlans = new LongAdder();
    private final LongAdder stalePlans = new LongAdder();
    private final LongAdder cacheHits = new LongAdder();
    private final LongAdder cachePuts = new LongAdder();
    private final LongAdder handleCancellations = new LongAdder();
    private final LongAdder handleShedPlans = new LongAdder();
    private final ConcurrentLinkedQueue<Throwable> unexpected =
            new ConcurrentLinkedQueue<>();
    private final AtomicInteger unexpectedCount = new AtomicInteger();
    private final List<Sample> samples = new ArrayList<>();
    private final Series tickNanos = new Series();
    private final Series periodNanos = new Series();
    private final Series coordinatorNanos = new Series();
    private final Series envNanos = new Series();
    private final Series dispatchNanos = new Series();

    LoadMetrics() {
        for (NavigationState state : NavigationState.values()) {
            travelTerminals.put(state, new LongAdder());
        }
    }

    /** One adaptivePlan() call, so the library's own total can be checked. */
    void adaptiveCall() {
        adaptiveCalls.increment();
    }

    /** Only episodes that actually had ground to cover are counted. */
    void travelTerminal(NavigationState state) {
        travelTerminals.get(state).increment();
        travelEpisodes.increment();
    }

    void handleCancelled() {
        handleCancellations.increment();
    }

    /** One adaptive plan that came back shed by a full worker queue. */
    void handleShed() {
        handleShedPlans.increment();
    }

    void unusablePlan() {
        unusablePlans.increment();
    }

    void stalePlan() {
        stalePlans.increment();
    }

    void cacheHit() {
        cacheHits.increment();
    }

    void cachePut() {
        cachePuts.increment();
    }

    void unexpected(Throwable failure) {
        if (unexpectedCount.incrementAndGet() <= 8) unexpected.add(failure);
    }

    void sample(Sample sample) {
        samples.add(sample);
    }

    void tickTook(long nanos) {
        tickNanos.add(nanos);
    }

    /** Work plus pacing: what one modelled tick actually cost end to end. */
    void tickPeriodTook(long nanos) {
        periodNanos.add(nanos);
    }

    void coordinatorTickTook(long nanos) {
        coordinatorNanos.add(nanos);
    }

    void envTickTook(long nanos) {
        envNanos.add(nanos);
    }

    /** Wall time of one adaptivePlan() call, measured on the tick thread. */
    void dispatchTook(long nanos) {
        dispatchNanos.add(nanos);
    }

    long adaptiveCalls() {
        return adaptiveCalls.sum();
    }

    long travelTerminals(NavigationState state) {
        return travelTerminals.get(state).sum();
    }

    long travelEpisodes() {
        return travelEpisodes.sum();
    }

    /** Stricter than the library rate: trivial already-at-goal ends removed. */
    double travelStuckFraction() {
        long total = travelEpisodes.sum();
        return total == 0 ? 0
                : (double) travelTerminals(NavigationState.STUCK) / total;
    }

    long handleCancellations() {
        return handleCancellations.sum();
    }

    long handleShedPlans() {
        return handleShedPlans.sum();
    }

    long stalePlans() {
        return stalePlans.sum();
    }

    long unusablePlans() {
        return unusablePlans.sum();
    }

    long cacheHits() {
        return cacheHits.sum();
    }

    long cachePuts() {
        return cachePuts.sum();
    }

    int unexpectedCount() {
        return unexpectedCount.get();
    }

    List<Throwable> unexpectedSamples() {
        return List.copyOf(unexpected);
    }

    List<Sample> samples() {
        return List.copyOf(samples);
    }

    double medianTickPeriodMillis() {
        return periodNanos.percentiles()[0] / 1e6;
    }

    int peak(java.util.function.ToIntFunction<Sample> field) {
        int peak = 0;
        for (Sample sample : samples) {
            peak = Math.max(peak, field.applyAsInt(sample));
        }
        return peak;
    }

    /** An unsynchronized wall-time series; only the tick thread appends. */
    private static final class Series {
        private final List<Long> values = new ArrayList<>();

        void add(long nanos) {
            values.add(nanos);
        }

        long[] percentiles() {
            if (values.isEmpty()) return new long[]{0, 0, 0, 0};
            long[] sorted = values.stream().mapToLong(Long::longValue)
                    .sorted().toArray();
            int count = sorted.length;
            return new long[]{
                    sorted[count / 2],
                    sorted[Math.min(count - 1, (int) (count * 0.95))],
                    sorted[Math.min(count - 1, (int) (count * 0.99))],
                    sorted[count - 1]};
        }
    }

    void printSummary(
            String label, long seed, int entities, long ticks,
            double wallSeconds, long heapBefore, long heapAfter,
            int queueCapacity, LoadClock clock,
            NavigationMetricsSnapshot warm,
            NavigationMetricsSnapshot end) {
        NavigationMetricsSnapshot.SearchCounts searches = end.searches();
        LatencyDistribution latency = end.latency().since(warm.latency());
        LatencyDistribution wait = end.queueWait().since(warm.queueWait());
        long[] tick = tickNanos.percentiles();
        long[] period = periodNanos.percentiles();
        long[] coordinator = coordinatorNanos.percentiles();
        long[] world = envNanos.percentiles();
        long[] dispatch = dispatchNanos.percentiles();
        long windowSearches =
                searches.submitted() - warm.searches().submitted();
        double window = end.elapsedSecondsSince(warm);

        StringBuilder text = new StringBuilder(2048);
        text.append(String.format(Locale.ROOT,
                "%n===== NAVIGATION LOAD %s =====%n", label));
        text.append(String.format(Locale.ROOT,
                "seed=%d entities=%d ticks=%d wall=%.2fs "
                        + "parallelism=%d queueCapacity=%d%n",
                seed, entities, ticks, wallSeconds,
                end.queue().parallelism(), queueCapacity));
        text.append(String.format(Locale.ROOT,
                "searches submitted=%d completed=%d failed=%d cancelled=%d "
                        + "rejected=%d superseded=%d inFlight=%d%n",
                searches.submitted(), searches.completed(), searches.failed(),
                searches.cancelled(), searches.rejected(),
                searches.superseded(), searches.inFlight()));
        text.append(String.format(Locale.ROOT,
                "steady window: %d searches in %.2fs = %.1f/s%n",
                windowSearches, window,
                windowSearches / Math.max(1e-9, window)));
        // Completions per tick is the quantity a free-running loop distorts:
        // it moves when the tick thread changes speed even though the pool
        // finished exactly as much work per second.
        text.append(String.format(Locale.ROOT,
                "throughput %.2f completions/tick over %d ticks = %.1f/s%n",
                searches.completed() / (double) Math.max(1, ticks), ticks,
                searches.completed() / Math.max(1e-9, wallSeconds)));
        text.append(String.format(Locale.ROOT,
                "latency   p50<=%.2fms p90<=%.2fms p99<=%.2fms max=%.2fms "
                        + "mean=%.2fms n=%d%n",
                latency.p50Micros() / 1e3, latency.p90Micros() / 1e3,
                latency.p99Micros() / 1e3, latency.maximumMicros() / 1e3,
                latency.meanMicros() / 1e3, latency.count()));
        text.append(String.format(Locale.ROOT,
                "queueWait p50<=%.2fms p90<=%.2fms p99<=%.2fms max=%.2fms n=%d%n",
                wait.p50Micros() / 1e3, wait.p90Micros() / 1e3,
                wait.p99Micros() / 1e3, wait.maximumMicros() / 1e3,
                wait.count()));
        text.append(String.format(Locale.ROOT,
                "tick wall p50=%.2fms p95=%.2fms p99=%.2fms max=%.2fms%n",
                tick[0] / 1e6, tick[1] / 1e6, tick[2] / 1e6, tick[3] / 1e6));
        text.append(String.format(Locale.ROOT,
                "tick period p50=%.2fms p95=%.2fms p99=%.2fms max=%.2fms "
                        + "clock=%s overruns=%d/%d%n",
                period[0] / 1e6, period[1] / 1e6, period[2] / 1e6,
                period[3] / 1e6,
                clock.paced()
                        ? String.format(Locale.ROOT, "%.0fms",
                                clock.periodMillis())
                        : "free-running",
                clock.overruns(), clock.ticks()));
        text.append(String.format(Locale.ROOT,
                "  adaptive tick   p50=%.3fms p95=%.3fms p99=%.3fms max=%.3fms%n",
                coordinator[0] / 1e6, coordinator[1] / 1e6,
                coordinator[2] / 1e6, coordinator[3] / 1e6));
        text.append(String.format(Locale.ROOT,
                "  instance tick   p50=%.3fms p95=%.3fms p99=%.3fms max=%.3fms%n",
                world[0] / 1e6, world[1] / 1e6, world[2] / 1e6, world[3] / 1e6));
        text.append(String.format(Locale.ROOT,
                "  adaptivePlan()  p50=%.3fms p95=%.3fms p99=%.3fms max=%.3fms%n",
                dispatch[0] / 1e6, dispatch[1] / 1e6, dispatch[2] / 1e6,
                dispatch[3] / 1e6));
        NavigationMetricsSnapshot.AdaptiveSourceCounts sources =
                end.adaptiveSources();
        text.append(String.format(Locale.ROOT,
                "adaptive individual=%d shared=%d parity=%d untracked=%d "
                        + "total=%d sharedHitRate=%.4f%n",
                sources.individualSearch(), sources.sharedTargetField(),
                sources.parityCertificationSearch(),
                sources.untrackedFallback(), sources.total(),
                sources.sharedHitRate()));
        NavigationMetricsSnapshot.ControllerOutcomeCounts outcomes =
                end.controllerOutcomes();
        text.append(String.format(Locale.ROOT,
                "controller completed=%d stuck=%d failed=%d cancelled=%d "
                        + "total=%d stallRate=%.4f travelling=%d "
                        + "travelStallRate=%.4f%n",
                outcomes.completed(), outcomes.stuck(), outcomes.failed(),
                outcomes.cancelled(), outcomes.total(), outcomes.stallRate(),
                outcomes.travelling(), outcomes.travelStallRate()));
        text.append(String.format(Locale.ROOT,
                "harness travel episodes=%d travelStuck=%d fraction=%.4f%n",
                travelEpisodes(), travelTerminals(NavigationState.STUCK),
                travelStuckFraction()));
        text.append(String.format(Locale.ROOT,
                "handle futures: cancelled=%d shed=%d unexpected=%d%n",
                handleCancellations(), handleShedPlans(), unexpectedCount()));
        text.append(String.format(Locale.ROOT,
                "follower: stalePlans=%d (library %d) unusablePlans=%d "
                        + "planCacheHits=%d planCachePuts=%d%n",
                stalePlans(), end.stalePlans(), unusablePlans(), cacheHits(),
                cachePuts()));
        text.append(String.format(Locale.ROOT,
                "peak regions=%d targets=%d retainedNodes=%d "
                        + "coordinatorActors=%d queued=%d%n",
                peak(Sample::regions), peak(Sample::targets),
                peak(Sample::retainedNodes), peak(Sample::coordinatorActors),
                peak(Sample::queued)));
        text.append(String.format(Locale.ROOT,
                "peak plan maps observed=%d certified=%d published=%d "
                        + "final=%d/%d/%d%n",
                peak(Sample::observedPlans), peak(Sample::certifiedPlans),
                peak(Sample::publishedPlans),
                samples.isEmpty() ? 0 : samples.getLast().observedPlans(),
                samples.isEmpty() ? 0 : samples.getLast().certifiedPlans(),
                samples.isEmpty() ? 0 : samples.getLast().publishedPlans()));
        text.append(String.format(Locale.ROOT,
                "peak threads=%d pathfindingThreads=%d planCache=%d%n",
                peak(Sample::threads), peak(Sample::pathfindingThreads),
                peak(Sample::planCache)));
        text.append(String.format(Locale.ROOT,
                "heap beforeGC=%.1fMiB afterGC=%.1fMiB (reported, not asserted:"
                        + " one JVM's heap is far too noisy to bound)%n",
                heapBefore / 1048576.0, heapAfter / 1048576.0));
        if (!samples.isEmpty()) {
            text.append("samples tick/regions/targets/nodes/actors/"
                    + "planCache/threads/inFlight:\n");
            int step = Math.max(1, samples.size() / 12);
            for (int index = 0; index < samples.size(); index += step) {
                appendSample(text, samples.get(index), "");
            }
            appendSample(text, samples.getLast(), " (final)");
        }
        text.append("=====================================\n");
        System.out.print(text);
        System.out.flush();
    }

    private static void appendSample(
            StringBuilder text, Sample sample, String suffix) {
        text.append(String.format(Locale.ROOT,
                "  t=%-6d r=%-4d tg=%-5d nd=%-6d ac=%-5d pc=%-5d th=%-4d "
                        + "inflight=%d%s%n",
                sample.tick(), sample.regions(), sample.targets(),
                sample.retainedNodes(), sample.coordinatorActors(),
                sample.planCache(), sample.threads(),
                Math.max(0, sample.submitted() - sample.terminated()), suffix));
    }
}
