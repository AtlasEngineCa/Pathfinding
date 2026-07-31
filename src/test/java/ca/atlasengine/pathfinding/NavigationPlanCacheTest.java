package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.NavigationPlanCache;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class NavigationPlanCacheTest {
    private static final BoundingBox BOX = new BoundingBox(0.6, 1.8, 0.6);

    @Test
    void immutablePlanIsReusedOnlyForMatchingWorldRevisionAndStart() {
        TestWorld world = new TestWorld().floor(-4, 16, -4, 4, 0, Block.STONE);
        NavigationRequest request = NavigationRequest.builder(world,
                        new Pos(0.5, 1, 0.5), new Pos(10.5, 1, 0.5), BOX,
                        BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE))
                .maxPathLength(20)
                .build();
        PathResult result = new EntityPathfinder().findPath(
                request, SearchControl.NONE);
        NavigationPlan plan = NavigationPlan.from(request, result);
        NavigationPlanCache<String> cache = new NavigationPlanCache<>(2);

        cache.put("patrol:a-to-b", 7, plan);

        assertSame(plan, cache.get("patrol:a-to-b", 7).orElseThrow());
        assertTrue(cache.get("patrol:a-to-b", 7,
                new Pos(0.6, 1, 0.5), 0.25).isPresent());
        assertTrue(cache.get("patrol:a-to-b", 7,
                new Pos(3.5, 1, 0.5), 0.25).isEmpty());
        assertTrue(cache.get("patrol:a-to-b", 8).isEmpty(),
                "world revision change must invalidate stale geometry");
        assertEquals(0, cache.size());
        assertThrows(UnsupportedOperationException.class,
                () -> plan.nodes().add(plan.nodes().getFirst()));
    }

    @Test
    void cacheEvictsLeastRecentlyUsedPlan() {
        NavigationPlanCache<String> cache = new NavigationPlanCache<>(2);
        NavigationPlan plan = simplePlan();
        cache.put("a", 1, plan);
        cache.put("b", 1, plan);
        assertTrue(cache.get("a", 1).isPresent());
        cache.put("c", 1, plan);

        assertTrue(cache.get("a", 1).isPresent());
        assertTrue(cache.get("b", 1).isEmpty());
        assertTrue(cache.get("c", 1).isPresent());
    }

    private static NavigationPlan simplePlan() {
        return new NavigationPlan(new Pos(0.5, 1, 0.5),
                new Pos(1.5, 1, 0.5), BOX,
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE),
                PathStatus.FOUND,
                java.util.List.of(
                        new PathNode(0.5, 1, 0.5, PathNode.Movement.WALK),
                        new PathNode(1.5, 1, 0.5, PathNode.Movement.WALK)),
                2, 8);
    }
}
