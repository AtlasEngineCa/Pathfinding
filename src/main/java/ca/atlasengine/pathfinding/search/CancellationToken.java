package ca.atlasengine.pathfinding.search;

/**
 * A lightweight cooperative cancellation signal.
 */
@FunctionalInterface
public interface CancellationToken {
    CancellationToken NONE = () -> false;

    boolean isCancelled();
}
