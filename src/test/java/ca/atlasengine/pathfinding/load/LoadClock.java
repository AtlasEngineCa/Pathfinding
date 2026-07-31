package ca.atlasengine.pathfinding.load;

import java.util.concurrent.locks.LockSupport;

/**
 * Paces the harness loop to the server tick it models.
 *
 * <p>A free-running loop makes searches-completed-per-tick a function of JVM
 * speed rather than of the worker pool: the same pool finishing the same work
 * per second lands fewer searches per tick as soon as the tick thread gets
 * faster, which silently rescales every invariant expressed per tick. Pacing
 * the loop to fifty milliseconds is what makes a tick mean a tick.</p>
 *
 * <p>A zero period restores the free-running loop, which is how the confound
 * is reproduced rather than merely asserted.</p>
 */
final class LoadClock {
    /** Below this the park is replaced by a spin, which lands far closer. */
    private static final long SPIN_NANOS = 200_000;

    private final long periodNanos;

    private long deadline;
    private int overruns;
    private int ticks;

    private LoadClock(long periodNanos) {
        this.periodNanos = periodNanos;
    }

    static LoadClock ofMillis(long millis) {
        return new LoadClock(Math.max(0, millis) * 1_000_000L);
    }

    boolean paced() {
        return periodNanos > 0;
    }

    double periodMillis() {
        return periodNanos / 1e6;
    }

    int overruns() {
        return overruns;
    }

    int ticks() {
        return ticks;
    }

    void start() {
        deadline = System.nanoTime() + periodNanos;
    }

    /**
     * Parks until this tick's slot ends, returning the wall clock at that
     * point. A tick that overran its slot is counted and the schedule restarts
     * from now, so a slow tick is never repaid as a burst of unpaced ticks the
     * way a fixed-rate schedule would repay it. A real server lags; it does not
     * run the next second of ticks at once.
     */
    long awaitTick() {
        ticks++;
        long now = System.nanoTime();
        if (periodNanos == 0) return now;
        if (now >= deadline) {
            overruns++;
            deadline = now + periodNanos;
            return now;
        }
        while (true) {
            long remaining = deadline - now;
            if (remaining <= 0) break;
            if (remaining > SPIN_NANOS) {
                LockSupport.parkNanos(remaining - SPIN_NANOS);
            } else Thread.onSpinWait();
            now = System.nanoTime();
        }
        deadline += periodNanos;
        return now;
    }
}
