package ca.atlasengine.pathfinding.result;

/**
 * The outcome of a path search.
 */
public enum PathStatus {
    FOUND,
    PARTIAL,
    CANCELLED,
    TIMED_OUT,
    INVALID_REQUEST,
    /**
     * The bounded worker queue was full, so the search was shed and never
     * ran. Routine backpressure rather than a failure: the caller keeps
     * whatever route it already has and asks again later.
     */
    SHED
}
