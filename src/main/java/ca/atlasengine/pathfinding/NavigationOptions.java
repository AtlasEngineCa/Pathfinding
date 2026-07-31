package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.result.PathResult;
/**
 * The settings a {@link NavigationSystem} was built with, read back as one
 * value.
 *
 * @param parallelism worker threads owned by the search service
 * @param queueCapacity bound on queued searches; submissions beyond it are
 *                      shed with {@link PathResult#SHED}
 * @param maximumConcurrentControllerSearches controller searches admitted to
 *                                            the worker layer at once
 * @param maximumDeferredControllerRequests latest controller intents retained
 *                                          outside the executor queue
 * @param minimumSearchStallTicks lower bound before a pending entity search is
 *                                assumed lost and replaced
 * @param shedBackoffTicks delay before an entity retries scheduler-shed work
 */
public record NavigationOptions(
        int parallelism,
        int queueCapacity,
        int maximumConcurrentControllerSearches,
        int maximumDeferredControllerRequests,
        int minimumSearchStallTicks,
        int shedBackoffTicks
) {
}
