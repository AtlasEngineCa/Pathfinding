package ca.atlasengine.pathfinding.adaptive;

import ca.atlasengine.pathfinding.NavigationStrategy;

/**
 * One immutable reading of a {@link SharedMeshNavigation}: whether it is
 * switched on, the policy it enforces, what it currently retains, and how much
 * of its traffic it answered.
 *
 * <p>{@code meshRequests} counts requests that consulted the mesh and {@code
 * sharedPlans} the subset it answered from a certified target field. {@code
 * bypassedRequests} counts requests planned individually because the caller
 * chose {@link ca.atlasengine.pathfinding.NavigationStrategy#INDIVIDUAL_ONLY} or
 * the mesh was disabled; those never entered a region, so they are excluded
 * from the hit rate below.</p>
 */
public record SharedMeshStatus(
        boolean enabled,
        SharedMeshPolicy policy,
        SharedMeshRetention regions,
        long meshRequests,
        long sharedPlans,
        long bypassedRequests) {

    public SharedMeshStatus {
        if (policy == null || regions == null) {
            throw new IllegalArgumentException("policy and regions");
        }
    }

    /** Fraction of mesh-consulting requests answered from a target field. */
    public double sharedHitRate() {
        return meshRequests == 0 ? 0 : (double) sharedPlans / meshRequests;
    }

    /**
     * Whether the region table is full. Regions partition on the whole request
     * semantics, including the live grounded and in-water flags, so a diverse
     * population reaches this bound and every further request degrades to
     * {@link SharedMeshSource#UNTRACKED_FALLBACK}: ordinary A* plus
     * the coordinator's bookkeeping. A mesh that is permanently exhausted is
     * costing more than it saves.
     */
    public boolean regionsExhausted() {
        return regions.regions() >= policy.maximumRegions();
    }
}
