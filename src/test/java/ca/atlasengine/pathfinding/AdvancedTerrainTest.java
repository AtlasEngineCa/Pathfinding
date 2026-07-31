package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

class AdvancedTerrainTest {
    private final DiscreteGroundPathfinder engine = new DiscreteGroundPathfinder();

    @ParameterizedTest
    @ValueSource(doubles = {0.4, 0.8, 1.4})
    void multiLevelRidgeWorksForSeveralBoundingBoxes(double width) {
        TestWorld world = flat(18);
        for (int x = -18; x <= 18; x++) {
            world.column(x, -2, 1, 6, Block.STONE);
            world.column(x, 2, 1, 6, Block.STONE);
        }
        for (int z = -1; z <= 1; z++) {
            world.set(-3, 1, z, Block.STONE);
            for (int y = 1; y <= 2; y++) {
                for (int x = -2; x <= 1; x++) world.set(x, y, z, Block.STONE);
            }
            world.set(2, 1, z, Block.STONE);
        }
        BoundingBox box = new BoundingBox(width, 1.8, width);

        PathResult result = find(world, new Pos(-6.5, 1, 0.5),
                new Pos(6.5, 1, 0.5), box, 40);

        assertTrue(result.found(), () -> width + ": " + result);
        assertTrue(result.nodes().stream().mapToDouble(PathNode::y)
                .max().orElse(0) >= 3);
        assertTrue(verticalChange(result) >= 4 - 1.0e-3,
                () -> width + " climbed and descended only "
                        + verticalChange(result) + " blocks");
    }

    private static double verticalChange(PathResult result) {
        double change = 0;
        for (int i = 1; i < result.nodes().size(); i++) {
            change += Math.abs(result.nodes().get(i).y()
                    - result.nodes().get(i - 1).y());
        }
        return change;
    }

    private PathResult find(TestWorld world, Pos start, Pos goal,
                            BoundingBox box, double maxLength) {
        return engine.findPath(world, start, goal, box, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(maxLength).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
    }

    private static TestWorld flat(int radius) {
        return new TestWorld().floor(-radius, radius, -radius, radius, 0, Block.STONE);
    }
}
