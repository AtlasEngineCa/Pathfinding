package ca.atlasengine.pathfinding.metrics;

/**
 * Why one follower's armed route splices did or did not become joined routes.
 *
 * <p>A splice plans a replacement route from where the entity will be when
 * the search lands, then joins it to the route still being walked.
 * {@code overrun} counts the entities that passed the splice node before their
 * plan landed, {@code expired} those whose search outran its own predicted
 * wait, {@code superseded} the routes replaced while the search ran,
 * {@code seamMismatch} the plans that did not start where they were asked to,
 * and {@code shortRoute} and {@code divergent} the replans that never armed
 * because the route was too short or headed away from the new target. Each is
 * a fallback to replanning from the current position, never a discarded
 * route.</p>
 */
public record SpliceCounts(long armed, long joined, long seamMismatch,
                           long overrun, long expired, long superseded,
                           long shortRoute, long divergent) {
}
