package ca.atlasengine.pathfinding.adaptive;

import ca.atlasengine.pathfinding.NavigationSystem;import ca.atlasengine.pathfinding.search.EntityTraversalState;
/**
 * The explicit opt-in and policy for shared-mesh planning.
 *
 * <p>A {@link ca.atlasengine.pathfinding.NavigationSystem} plans individually
 * unless it is built with an enabled instance of this record, so the mesh is
 * never reached by accident. Build one from operator configuration to keep the
 * choice switchable without a rebuild; {@link
 * SharedMeshNavigation#disable()} switches it off again at runtime.</p>
 *
 * <p>Enable it knowingly. A region is keyed on the whole request semantics:
 * the world, the regional cell of the start, the bounding box, the profile,
 * the ordered influence snapshot, the sea level, every one of the four search
 * limits, and the build-height bounds of the live {@link
 * ca.atlasengine.pathfinding.search.EntityTraversalState}. Its remaining components
 * resolve only the start node and key the source instead, so one mob keeps
 * its region as it jumps or swims. A diverse population can still reach
 * {@link SharedMeshPolicy#maximumRegions()}, and every further
 * request then degrades to {@link
 * SharedMeshSource#UNTRACKED_FALLBACK}: ordinary A* plus the
 * coordinator's bookkeeping. Watch {@link
 * SharedMeshStatus#regionsExhausted()}.</p>
 *
 * <p>The win is real but bounded. On a small fixture a mesh-served request
 * measured about 0.1-0.2 ms against about 1.2 ms for the individual search it
 * replaces, and the request that triggers promotion pays a one-off hitch of
 * about 8.1 ms. Traffic that never repeats a source pays that hitch and keeps
 * searching; see the wiki for what shares.</p>
 */
public record SharedMeshOptions(
        boolean enabled, SharedMeshPolicy policy) {
    /** The default: every request is planned individually. */
    public static final SharedMeshOptions DISABLED =
            new SharedMeshOptions(false, SharedMeshPolicy.DEFAULT);

    /** Opts in with the default promotion and lifecycle policy. */
    public static final SharedMeshOptions ENABLED =
            new SharedMeshOptions(true, SharedMeshPolicy.DEFAULT);

    public SharedMeshOptions {
        if (policy == null) throw new IllegalArgumentException("policy");
    }

    /** Opts in with an explicit promotion and lifecycle policy. */
    public static SharedMeshOptions enabledWith(
            SharedMeshPolicy policy) {
        return new SharedMeshOptions(true, policy);
    }
}
