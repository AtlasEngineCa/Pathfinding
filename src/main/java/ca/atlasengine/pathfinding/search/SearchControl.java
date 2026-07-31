package ca.atlasengine.pathfinding.search;

import java.time.Duration;

/**
 * Cooperative cancellation and monotonic deadline for one search.
 */
public record SearchControl(CancellationToken cancellation, long deadlineNanos) {
    public static final SearchControl NONE =
            new SearchControl(CancellationToken.NONE, Long.MAX_VALUE);

    public SearchControl {
        if (cancellation == null) throw new IllegalArgumentException("cancellation cannot be null");
    }

    public static SearchControl withTimeout(CancellationToken cancellation, Duration timeout) {
        if (timeout == null || timeout.isNegative() || timeout.isZero()) {
            throw new IllegalArgumentException("timeout must be positive");
        }
        long now = System.nanoTime();
        long duration = timeout.toNanos();
        long deadline = duration >= Long.MAX_VALUE - now ? Long.MAX_VALUE : now + duration;
        return new SearchControl(cancellation, deadline);
    }

    public boolean cancelled() {
        return cancellation.isCancelled();
    }

    public boolean timedOut() {
        return System.nanoTime() >= deadlineNanos;
    }

    /** Whether inner evaluator loops need periodic cooperative polling. */
    public boolean interruptible() {
        return cancellation != CancellationToken.NONE
                || deadlineNanos != Long.MAX_VALUE;
    }
}
