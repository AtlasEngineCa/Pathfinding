package ca.atlasengine.pathfinding.adaptive;

import ca.atlasengine.pathfinding.NavigationStrategy;

/** How an adaptive request obtained its follower-ready plan. */
public enum SharedMeshSource {
    /** A normal asynchronous A* request, also eligible to seed later reuse. */
    INDIVIDUAL_SEARCH,
    /** A route from an already built regional target field. */
    SHARED_TARGET_FIELD,
    /**
     * A normal A* request required because a promoted mesh had no exact
     * individual certificate or failed semantic parity validation.
     */
    PARITY_CERTIFICATION_SEARCH,
    /** A normal A* request not retained because the coordinator is at capacity. */
    UNTRACKED_FALLBACK,
    /**
     * A normal A* request that never consulted the mesh, because the caller
     * chose {@link ca.atlasengine.pathfinding.NavigationStrategy#INDIVIDUAL_ONLY}
     * or the mesh is disabled. Unlike {@link #INDIVIDUAL_SEARCH} it joins no
     * region and seeds no later reuse.
     */
    UNSHARED_SEARCH
}
