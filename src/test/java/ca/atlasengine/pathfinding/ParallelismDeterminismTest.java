package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

class ParallelismDeterminismTest {
    @Test
    void workerCountDoesNotChangeSearchResults() {
        TestWorld world = new TestWorld().floor(-4, 18, -8, 8, 0, Block.STONE);
        for (int x : new int[]{4, 8, 12}) {
            int gate = x == 8 ? -3 : 3;
            for (int z = -6; z <= 6; z++) {
                if (z == gate) continue;
                for (int y = 1; y <= 4; y++) world.set(x, y, z, Block.STONE);
            }
        }
        NavigationRequest request = NavigationRequest.builder(world,
                        new Pos(0.5, 1, 0.5), new Pos(16.5, 1, 0.5),
                        new BoundingBox(0.6, 1.8, 0.6),
                        BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE))
                .maxPathLength(32).build();

        PathResult serial;
        try (var service = new AsyncEntityPathfindingService(1, 64)) {
            serial = service.submit(request).join();
        }
        List<PathResult> parallel = new ArrayList<>();
        try (var service = new AsyncEntityPathfindingService(4, 64)) {
            for (int index = 0; index < 32; index++) {
                parallel.add(service.submit(request).join());
            }
        }
        for (int index = 0; index < parallel.size(); index++) {
            PathResult result = parallel.get(index);
            assertEquals(serial.status(), result.status(), "status run=" + index);
            assertEquals(serial.stop(), result.stop(), "stop run=" + index);
            assertEquals(serial.nodes(), result.nodes(), "nodes run=" + index);
        }
    }
}
