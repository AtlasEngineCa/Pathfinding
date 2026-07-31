package ca.atlasengine.pathfinding.result;

/**
 * Why a search stopped. {@link PathStatus} says what a search produced; this
 * says what ended it.
 *
 * <p>{@link PathStatus#PARTIAL} conflates two outcomes a caller must tell
 * apart. A frontier that emptied proves the target is unreachable from the
 * start the search was given. A frontier that was cut short proves only that a
 * budget ran out, and searching on from further along the route makes progress.
 * This enum is that distinction, and it is carried on {@link PathResult#stop()}
 * beside the status rather than folded into it, so no existing status match
 * changes meaning.</p>
 */
public enum PathStop {
    /** A destination was reached. */
    REACHED,
    /**
     * The open set emptied with nothing withheld from it, so every cell
     * reachable from the start was expanded and none was a destination.
     * Searching again from the same place cannot find one.
     */
    FRONTIER_EXHAUSTED,
    /**
     * The search stopped with the open set still holding cells, because the
     * visit budget, the deadline, or a cancellation ended it first. A further
     * search expands graph this one never saw.
     */
    FRONTIER_TRUNCATED,
    /**
     * The open set emptied only because the path-length bound withheld cells
     * from it. This proves nothing about reachability: the bound is measured
     * from the start, so a search begun further along the route enters cells
     * this one refused.
     */
    LENGTH_BOUNDED,
    /**
     * No graph search produced this result. Shed, invalid, and synthesised
     * results read this, as does any route replayed from a cached plan.
     */
    NOT_SEARCHED
}
