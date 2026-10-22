package ca.atlasengine.pathfinding.internal;

import ca.atlasengine.pathfinding.metrics.NavigationMetrics;

/** Converts observed search latency into the tick clock followers run on. */
public final class SearchLatency {
    private static final long MICROS_PER_TICK = 50_000;
    /** Multiple of the observed p99 after which a search is presumed lost. */
    private static final long STALL_TAIL_MULTIPLIER = 4;

    private SearchLatency() {
    }

    /** Whole ticks spanned by a latency in microseconds. */
    public static long ticks(long micros) {
        return micros / MICROS_PER_TICK;
    }

    /**
     * Whether a search that has been in flight for {@code waitedTicks} will
     * never land. Work in flight is left alone until the tail the system
     * actually observes says so, so a chased mob keeps receiving routes however
     * slow searches become. The tail is read only once the floor has already
     * elapsed, so the ordinary path costs nothing.
     */
    public static boolean abandoned(long waitedTicks, int minimumStallTicks,
                                    NavigationMetrics metrics) {
        if (waitedTicks < minimumStallTicks) return false;
        long tail = ticks(metrics.snapshot().latency().p99Micros());
        return waitedTicks >= Math.max(minimumStallTicks,
                tail * STALL_TAIL_MULTIPLIER);
    }
}
