package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.NavigationRequest;import java.util.concurrent.CompletableFuture;

/** Package-local admission seam used only by entity controllers. */
@FunctionalInterface
interface ControllerRequestSubmitter {
    CompletableFuture<PathResult> submit(
            Object key, NavigationRequest request, int priority);
}
