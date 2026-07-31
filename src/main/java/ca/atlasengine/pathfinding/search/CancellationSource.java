package ca.atlasengine.pathfinding.search;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Owns a cancellation signal that can safely be shared across threads.
 */
public final class CancellationSource {
    private final AtomicBoolean cancelled = new AtomicBoolean();

    public CancellationToken token() {
        return cancelled::get;
    }

    public boolean cancel() {
        return cancelled.compareAndSet(false, true);
    }

    public boolean isCancelled() {
        return cancelled.get();
    }
}
