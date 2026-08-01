package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.result.PathStatus;
import ca.atlasengine.pathfinding.search.EntityPathfinder;
import ca.atlasengine.pathfinding.search.EntityTraversalState;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.search.SearchControl;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertNotEquals;

class BuildHeightBoundaryTest {
    @Test
    void groundSearchCannotStandAboveHighestBuildableBlock() {
        TestWorld world = new TestWorld().floor(-2, 6, -2, 2, 15,
                Block.STONE);
        NavigationRequest request = NavigationRequest.builder(world,
                        new Pos(0.5, 16, 0.5), new Pos(4.5, 16, 0.5),
                        new BoundingBox(0.6, 1.8, 0.6),
                        BuiltinNavigationProfiles.forEntityType(
                                EntityType.ZOMBIE))
                .entityState(new EntityTraversalState(true, false, Set.of(),
                        -16, 15, List.of()))
                .build();

        var result = new EntityPathfinder().findPath(request,
                SearchControl.NONE);

        assertNotEquals(PathStatus.FOUND, result.status(),
                "a ground route cannot place the mob wholly above max height");
    }
}
