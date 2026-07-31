package ca.atlasengine.pathfinding;

/**
 * Internal controller-planning limits assembled by {@link NavigationSystem}.
 *
 * <p>{@code minimumSearchStallTicks} floors the wait after which a search is
 * assumed never to land; the default three seconds is an order of magnitude
 * beyond any healthy completion tail, so the observed tail governs whenever the
 * system is that slow. {@code shedBackoffTicks} is a fixed interval
 * because a full queue leaves no search whose completion could pace the retry,
 * which is the one place backpressure makes a constant the right answer.</p>
 */
record ControllerSchedulingOptions(
        int maximumDeferredRequests,
        int maximumConcurrentSearches,
        int minimumSearchStallTicks,
        int shedBackoffTicks) {
    ControllerSchedulingOptions {
        if (maximumDeferredRequests <= 0 || maximumConcurrentSearches <= 0
                || minimumSearchStallTicks <= 0 || shedBackoffTicks <= 0) {
            throw new IllegalArgumentException("invalid controller scheduling options");
        }
    }

    static ControllerSchedulingOptions direct(int parallelism) {
        return new ControllerSchedulingOptions(
                1, parallelism, 60, 20);
    }
}
