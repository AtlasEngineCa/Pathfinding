package ca.atlasengine.pathfinding.adaptive;

/**
 * What a shared mesh is currently holding, reported by
 * {@link SharedMeshStatus#regions()}.
 *
 * <p>The endpoint count says nothing about the route maps beside it, so the
 * last three are the only view of the memory a region holds outside its mesh.
 * They fall to zero with the region that owns them.</p>
 *
 * @param regions live regions
 * @param promotedRegions regions that have crossed the promotion threshold
 * @param actors actors holding membership
 * @param targets destination cells being tracked
 * @param retainedNodes endpoints of the waypoint mesh only
 * @param observedPlans routes held as evidence before promotion
 * @param certifiedPlans routes a promoted region will vouch for
 * @param publishedPlans mesh compositions a follower may replay
 */
public record SharedMeshRetention(
        int regions, int promotedRegions, int actors,
        int targets, int retainedNodes, int observedPlans,
        int certifiedPlans, int publishedPlans) {
}
