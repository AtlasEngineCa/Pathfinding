package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.CancellationSource;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Engine-level cancellation. A mob whose replan is superseded can still follow
 * what the cancelled search already found, so cancelling mid-search returns the
 * best partial route rather than an empty one.
 */
class SearchCancellationTest {
    private static final BoundingBox BOX = new BoundingBox(0.8, 1.8, 0.8);
    /** {@code max(16, maxPathLength) * 16 * maxVisitedMultiplier}. */
    private static final int VISITED_BUDGET = 1600;
    private final CancellationSource source = new CancellationSource();

    @Test
    void cancellingMidSearchReturnsTheBestPartialRoute() {
        TestWorld world = flat();
        PathResult result = new DiscreteGroundPathfinder().findPath(cancellingAfter(world, 2000), new Pos(-30.5, 1, 0.5), new Pos(30.5, 1, 0.5), BOX, MobTraversalProfile.DEFAULT, GroundSearchLimits.ofPathLength(100), GroundCapabilities.STANDARD, control());

        assertEquals(PathStatus.CANCELLED, result.status());
        assertFalse(result.nodes().isEmpty());
        assertTrue(result.nodes().size() > 1, result::toString);
        assertTrue(result.visitedNodes() < VISITED_BUDGET);
        assertEquals(-31, result.nodes().getFirst().graphX());
        assertTrue(result.nodes().getLast().graphX() > -31,
                "cancellation returned no progress at all");
        assertConnected(result);
    }

    @Test
    void cancelledEntityRequestKeepsTheSamePartialRoute() {
        TestWorld world = flat();
        PathResult result = new EntityPathfinder().findPath(
                NavigationRequest.builder(cancellingAfter(world, 2000),
                                new Pos(-30.5, 1, 0.5), new Pos(30.5, 1, 0.5),
                                BOX, BuiltinNavigationProfiles.forEntityType(
                                        EntityType.ZOMBIE))
                        .maxPathLength(100)
                        .build(), control());

        assertEquals(PathStatus.CANCELLED, result.status());
        assertTrue(result.nodes().size() > 1, result::toString);
        assertEquals(-31, result.nodes().getFirst().graphX());
        assertConnected(result);
    }

    @Test
    void cancellingBeforeTheFirstExpansionStillReportsTheStartNode() {
        TestWorld world = flat();
        PathResult result = new DiscreteGroundPathfinder().findPath(cancellingAfter(world, 0), new Pos(-30.5, 1, 0.5), new Pos(30.5, 1, 0.5), BOX, MobTraversalProfile.DEFAULT, GroundSearchLimits.ofPathLength(100), GroundCapabilities.STANDARD, control());

        assertEquals(PathStatus.CANCELLED, result.status());
        assertEquals(1, result.nodes().size(), result::toString);
        assertEquals(-31, result.nodes().getFirst().graphX());
        assertEquals(0, result.visitedNodes());
    }

    @Test
    void cancellationInterruptsALargeVolumeClassification() {
        TestWorld world = new TestWorld();
        PathResult result = new EntityPathfinder().findPath(
                NavigationRequest.builder(cancellingAfter(world, 70),
                                new Pos(0.5, 1, 0.5), new Pos(30.5, 1, 0.5),
                                new BoundingBox(16, 8, 16),
                                BuiltinNavigationProfiles.forEntityType(
                                        EntityType.ALLAY))
                        .maxPathLength(64)
                        .build(), control());

        assertEquals(PathStatus.CANCELLED, result.status());
        assertTrue(world.reads() < 3_000,
                "volume scan ignored cancellation: " + world.reads());
    }

    private static void assertConnected(PathResult result) {
        for (int i = 1; i < result.nodes().size(); i++) {
            PathNode previous = result.nodes().get(i - 1);
            PathNode node = result.nodes().get(i);
            assertTrue(Math.abs(node.graphX() - previous.graphX()) <= 1
                            && Math.abs(node.graphY() - previous.graphY()) <= 1
                            && Math.abs(node.graphZ() - previous.graphZ()) <= 1,
                    result::toString);
        }
    }

    private Block.Getter cancellingAfter(TestWorld world, long reads) {
        return (x, y, z, condition) -> {
            if (world.reads() > reads) source.cancel();
            return world.getBlock(x, y, z, condition);
        };
    }

    private SearchControl control() {
        return new SearchControl(source.token(), Long.MAX_VALUE);
    }

    private static TestWorld flat() {
        return new TestWorld().floor(-40, 40, -40, 40, 0, Block.STONE);
    }
}
