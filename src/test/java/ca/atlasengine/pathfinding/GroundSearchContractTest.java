package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.CancellationToken;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.Executors;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Engine-level guarantees every ground search owes its caller: what a search
 * reports when it cannot reach the target, that an expired deadline buys no
 * search work, that two identical searches are byte-identical, that one
 * pathfinder instance is shared safely, and that the returned result is a
 * bounded, immutable value.
 */
class GroundSearchContractTest {
    private static final BoundingBox STANDARD =
            new BoundingBox(0.6, 1.8, 0.6);
    private static final double MAX_LENGTH = 32;
    private static final double VISITED_MULTIPLIER = 8;
    /** {@link DiscreteGroundPathfinder}'s derived expansion budget. */
    private static final int VISITED_BUDGET = (int) (Math.floor(
            Math.max(16, MAX_LENGTH) * 16) * VISITED_MULTIPLIER);

    private final DiscreteGroundPathfinder pathfinder =
            new DiscreteGroundPathfinder();

    /**
     * A goal walled off on every side cannot be reached. The discrete search
     * exhausts its open set long before the expansion budget, so the reported
     * PARTIAL route is the definitive answer rather than a truncated one.
     *
     * <p>The engine has a single non-reaching terminal status; it does not
     * distinguish an exhausted open set from an exhausted budget. The visited
     * count below is what separates them.</p>
     */
    @Test
    void sealedTargetIsNeverReachedAndExhaustsTheOpenSet() {
        TestWorld world = flat();
        for (int x = 3; x <= 5; x++) {
            for (int z = 3; z <= 5; z++) {
                if (x == 4 && z == 4) continue;
                world.column(x, z, 1, 3, Block.STONE);
            }
        }

        PathResult result = find(world, new Pos(0.5, 1, 0.5),
                new Pos(4.5, 1, 4.5), STANDARD);

        assertEquals(PathStatus.PARTIAL, result.status());
        assertFalse(result.found());
        assertTrue(result.nodes().stream().noneMatch(node ->
                        node.graphX() == 4 && node.graphZ() == 4),
                () -> "route entered the sealed cell: " + result.nodes());
        PathNode end = result.nodes().getLast();
        assertFalse(end.graphX() == 4 && end.graphY() == 1 && end.graphZ() == 4,
                () -> "route claimed the sealed target: " + result.nodes());
        assertTrue(result.visitedNodes() < VISITED_BUDGET,
                () -> "the open set must be exhausted rather than the "
                        + "expansion budget: visited=" + result.visitedNodes()
                        + " budget=" + VISITED_BUDGET);
    }

    /**
     * An already-expired deadline buys no search: zero expansions and zero
     * neighbour examinations. The evaluator still resolves its start node
     * before the driver's first deadline check, so a handful of world reads
     * remain; they must stay a small constant fraction of a real search.
     */
    @Test
    void expiredDeadlineStopsBeforeAnySearchWork() {
        TestWorld expiredWorld = flat();
        SearchControl expired = new SearchControl(
                CancellationToken.NONE, System.nanoTime() - 1);

        PathResult result = pathfinder.findPath(expiredWorld, new Pos(0.5, 1, 0.5), new Pos(5.5, 1, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(MAX_LENGTH).reachRange(0).maxVisitedMultiplier(VISITED_MULTIPLIER).build(), GroundCapabilities.STANDARD, expired);

        assertEquals(PathStatus.TIMED_OUT, result.status());
        assertEquals(0, result.visitedNodes());
        assertEquals(0, result.examinedNeighbors());
        assertEquals(1, result.nodes().size(),
                () -> "only the resolved start node may be reported: "
                        + result.nodes());

        TestWorld completedWorld = flat();
        PathResult completed = find(completedWorld, new Pos(0.5, 1, 0.5),
                new Pos(5.5, 1, 0.5), STANDARD);
        assertTrue(completed.found());
        assertTrue(expiredWorld.reads() * 4 < completedWorld.reads(),
                () -> "an expired deadline must not perform search reads: "
                        + expiredWorld.reads() + " of "
                        + completedWorld.reads());
    }

    /**
     * The same fixture searched twenty-five times produces the identical node
     * list and the identical expansion count. Nothing in the search may depend
     * on hash iteration order, identity, or wall-clock time.
     */
    @Test
    void repeatedSearchIsDeterministic() {
        TestWorld world = flat().column(0, 3, 1, 2, Block.STONE);
        PathResult expected = find(world, new Pos(0.5, 1, 0.5),
                new Pos(0.5, 1, 8.5), STANDARD);

        assertTrue(expected.found());
        for (int attempt = 0; attempt < 25; attempt++) {
            PathResult actual = find(world, new Pos(0.5, 1, 0.5),
                    new Pos(0.5, 1, 8.5), STANDARD);
            assertEquals(expected.status(), actual.status());
            assertEquals(expected.nodes(), actual.nodes());
            assertEquals(expected.visitedNodes(), actual.visitedNodes());
            assertEquals(expected.examinedNeighbors(),
                    actual.examinedNeighbors());
        }
    }

    /**
     * One {@link EntityPathfinder} and one {@link DiscreteGroundPathfinder}
     * serve sixty-four searches across eight threads. Neither holds
     * search-scoped state, so every concurrent answer equals the answer the
     * same instance gives alone.
     */
    @Test
    void onePathfinderInstanceServesManyConcurrentSearches() throws Exception {
        TestWorld world = flat();
        EntityPathfinder entityPathfinder = new EntityPathfinder();
        DiscreteGroundPathfinder discrete = new DiscreteGroundPathfinder();
        NavigationProfile zombie =
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
        List<PathResult> sequential = new ArrayList<>();
        for (int lane = -8; lane < 8; lane++) {
            sequential.add(entityRun(entityPathfinder, zombie, world, lane));
            sequential.add(discreteRun(discrete, world, lane));
        }

        try (var executor = Executors.newFixedThreadPool(8)) {
            var tasks = new ArrayList<Callable<PathResult>>();
            for (int i = 0; i < 64; i++) {
                int lane = i % 16 - 8;
                tasks.add(() -> entityRun(
                        entityPathfinder, zombie, world, lane));
                tasks.add(() -> discreteRun(discrete, world, lane));
            }
            List<PathResult> observed = new ArrayList<>();
            for (var future : executor.invokeAll(tasks)) {
                observed.add(future.get());
            }
            assertEquals(128, observed.size());
            for (int i = 0; i < observed.size(); i++) {
                PathResult actual = observed.get(i);
                PathResult reference = sequential.get(i % sequential.size());
                assertTrue(actual.found(), actual::toString);
                assertEquals(reference.nodes(), actual.nodes(),
                        "concurrent search diverged from its serial answer");
                assertEquals(reference.visitedNodes(), actual.visitedNodes());
            }
        }
    }

    /**
     * A straight twenty-block walk expands one node per cell and stays far
     * below the collision-query budget an unshaped floor should need.
     */
    @Test
    void straightSearchHasBoundedWorldReads() {
        TestWorld world = flat();

        PathResult result = find(world, new Pos(0.5, 1, 0.5),
                new Pos(20.5, 1, 0.5), STANDARD);

        assertTrue(result.found());
        assertTrue(result.visitedNodes() <= 21,
                () -> "unexpected A* expansion regression: "
                        + result.visitedNodes());
        assertTrue(world.reads() < 5_000,
                () -> "collision/world-read regression: " + world.reads());
    }

    /**
     * A result is a value: neither its waypoints nor its charged node costs
     * may be edited by whoever received it.
     */
    @Test
    void pathResultCollectionsAreImmutable() {
        PathResult result = find(flat(), new Pos(0.5, 1, 0.5),
                new Pos(2.5, 1, 0.5), STANDARD);

        assertTrue(result.found());
        assertThrows(UnsupportedOperationException.class,
                () -> result.nodes().add(
                        new PathNode(0, 0, 0, PathNode.Movement.WALK)));
        assertThrows(UnsupportedOperationException.class,
                () -> result.nodes().clear());
        assertThrows(UnsupportedOperationException.class,
                () -> result.nodeCosts().clear());
    }

    /**
     * A non-finite endpoint is a caller error, not a search: it is reported
     * before the world is touched and before the budget is validated.
     */
    @Test
    void nonFiniteEndpointsAreRejectedBeforeAnyWorldRead() {
        TestWorld world = flat();

        assertEquals(PathStatus.INVALID_REQUEST,
                find(world, new Pos(Double.NaN, 1, 0.5),
                        new Pos(2.5, 1, 0.5), STANDARD).status());
        assertEquals(PathStatus.INVALID_REQUEST,
                find(world, new Pos(0.5, 1, 0.5),
                        new Pos(2.5, Double.POSITIVE_INFINITY, 0.5),
                        STANDARD).status());
        assertEquals(0, world.reads());
        assertThrows(IllegalArgumentException.class,
                () -> pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(2.5, 1, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(MAX_LENGTH).reachRange(0).maxVisitedMultiplier(0).build(), GroundCapabilities.STANDARD, SearchControl.NONE));
    }

    private PathResult find(TestWorld world, Pos start, Pos goal,
                            BoundingBox box) {
        return pathfinder.findPath(world, start, goal, box, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(MAX_LENGTH).reachRange(0).maxVisitedMultiplier(VISITED_MULTIPLIER).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
    }

    private static PathResult entityRun(
            EntityPathfinder pathfinder, NavigationProfile profile,
            TestWorld world, int lane) {
        return pathfinder.findPath(NavigationRequest.builder(
                                world, new Pos(-10.5, 1, lane + 0.5),
                                new Pos(10.5, 1, lane + 0.5), STANDARD, profile)
                        .maxPathLength(MAX_LENGTH)
                        .maxVisitedMultiplier(VISITED_MULTIPLIER)
                        .build(),
                SearchControl.NONE);
    }

    private static PathResult discreteRun(
            DiscreteGroundPathfinder pathfinder, TestWorld world, int lane) {
        return pathfinder.findPath(world, new Pos(-10.5, 1, lane + 0.5), new Pos(10.5, 1, lane + 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(MAX_LENGTH).reachRange(0).maxVisitedMultiplier(VISITED_MULTIPLIER).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
    }

    private static TestWorld flat() {
        return new TestWorld().floor(-24, 24, -24, 24, 0, Block.STONE);
    }
}
