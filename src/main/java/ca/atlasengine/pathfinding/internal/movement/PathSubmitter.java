package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.profile.NavigationModifiers;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.Instance;

import java.util.List;
import java.util.concurrent.CompletableFuture;

/**
 * Seam between a controller and the search service it plans with.
 */
@FunctionalInterface
public interface PathSubmitter {
    CompletableFuture<PathResult> submit(
            Object key, Instance instance, Point start,
            Point target, BoundingBox boundingBox,
            List<NavigationInfluence> influences,
            NavigationModifiers modifiers);
}
