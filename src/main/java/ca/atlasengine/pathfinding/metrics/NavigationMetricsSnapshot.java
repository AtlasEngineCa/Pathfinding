package ca.atlasengine.pathfinding.metrics;

/**
 * One immutable reading of every navigation counter, gauge, and latency
 * distribution.
 *
 * <p>Counters are cumulative and never reset, so an operator computes rates by
 * differencing two snapshots and dividing by
 * {@code capturedAtNanos} elapsed. See {@link NavigationMetrics#snapshot()} for
 * the exact consistency guarantee.</p>
 *
 * <p>{@code stalePlans} counts plans a follower refused because its entity
 * moved beyond the start tolerance while the search ran. Those searches still
 * completed, so this is not a search outcome; a sustained rate means replans
 * are landing later than the mob moves.</p>
 */
public record NavigationMetricsSnapshot(
        long capturedAtNanos,
        SearchCounts searches,
        LatencyDistribution latency,
        LatencyDistribution queueWait,
        AdaptiveSourceCounts adaptiveSources,
        ControllerOutcomeCounts controllerOutcomes,
        long stalePlans,
        QueueState queue) {

    /** Seconds elapsed between an earlier snapshot and this one. */
    public double elapsedSecondsSince(NavigationMetricsSnapshot earlier) {
        if (earlier == null) throw new IllegalArgumentException("earlier");
        return (capturedAtNanos - earlier.capturedAtNanos) / 1_000_000_000.0;
    }

    /**
     * Cumulative search lifecycle counts. Every submitted search reaches
     * exactly one terminal counter.
     */
    public record SearchCounts(
            long submitted, long completed, long failed, long cancelled,
            long rejected, long superseded) {

        public long terminated() {
            return completed + failed + cancelled + rejected + superseded;
        }

        /** Searches queued or running when the snapshot was taken. */
        public long inFlight() {
            return Math.max(0, submitted - terminated());
        }
    }

    /**
     * Cumulative adaptive dispatch decisions, one per adaptive request.
     * {@code sharedTargetField} is the only outcome that answers a request
     * without an A* search. {@code unsharedSearch} counts the requests that
     * never consulted the mesh at all, because the caller forced individual
     * planning or the mesh was disabled.
     */
    public record AdaptiveSourceCounts(
            long individualSearch, long sharedTargetField,
            long parityCertificationSearch, long untrackedFallback,
            long unsharedSearch) {

        public long total() {
            return individualSearch + sharedTargetField
                    + parityCertificationSearch + untrackedFallback
                    + unsharedSearch;
        }

        /** Fraction of adaptive requests answered from the regional mesh. */
        public double sharedHitRate() {
            long total = total();
            return total == 0 ? 0 : (double) sharedTargetField / total;
        }
    }

    /**
     * Cumulative controller outcomes. One is recorded each time a controller
     * leaves an active navigation for a terminal state.
     *
     * <p>{@code travelling} re-counts the subset whose entity started outside
     * its own arrival tolerance, plus every stall, so it never exceeds
     * {@link #total()} and never omits a {@code stuck}.</p>
     *
     * <p>{@code unreachableTargets} is not one of the four outcomes and is not
     * in {@link #total()}. It counts navigations the planner stopped offering
     * segments for while the follower stayed healthy: the search cannot reach
     * the goal from where the entity stands, so the caller should choose
     * another goal. Those used to arrive as a {@code stuck} two hundred ticks
     * later, which is why this counter rising while {@code stuck} falls is the
     * expected shape rather than a regression.</p>
     */
    public record ControllerOutcomeCounts(
            long completed, long stuck, long failed, long cancelled,
            long travelling, long unreachableTargets) {

        /** Every finished navigation, whether or not it had to move. */
        public long total() {
            return completed + stuck + failed + cancelled;
        }

        /**
         * Fraction of every finished navigation that ended in a visible
         * stall. The denominator is {@link #total()}, which includes
         * navigations that began inside arrival tolerance and completed on
         * their first tick without moving; a mob standing on its goal and
         * replanning every second contributes one such outcome per replan.
         * Alarm on {@link #travelStallRate()} instead.
         */
        public double stallRate() {
            long total = total();
            return total == 0 ? 0 : (double) stuck / total;
        }

        /**
         * Fraction of navigations that had ground to cover and ended in a
         * visible stall. The denominator is {@link #travelling()}: finished
         * navigations whose entity started outside arrival tolerance of its
         * goal, which is every stall plus every other navigation that had to
         * move. The numerator is unchanged and the denominator can only
         * shrink, so this never reads below {@link #stallRate()}. This is the
         * rate an operator should alarm on.
         */
        public double travelStallRate() {
            return travelling == 0 ? 0 : (double) stuck / travelling;
        }
    }

    /** Instantaneous worker-pool state; zero when no service is bound. */
    public record QueueState(
            int activeSearches, int queuedSearches, int queueCapacity,
            int parallelism) {
    }
}
