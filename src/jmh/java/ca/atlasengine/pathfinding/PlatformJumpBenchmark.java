package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.instance.block.Block;
import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.BenchmarkMode;
import org.openjdk.jmh.annotations.Level;
import org.openjdk.jmh.annotations.Mode;
import org.openjdk.jmh.annotations.OutputTimeUnit;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.Setup;
import org.openjdk.jmh.annotations.State;

import java.util.concurrent.TimeUnit;

/**
 * Platform jumping is the only search extension whose edge test sweeps a
 * bounding box, so it is measured on worlds where every step of the route is
 * a jump candidate rather than on the shared long-route fixtures.
 */
@BenchmarkMode(Mode.AverageTime)
@OutputTimeUnit(TimeUnit.MICROSECONDS)
public class PlatformJumpBenchmark {
    @Benchmark
    public int steppingStones128(SteppingStones state) {
        return verify(state.engine.findPath(state.request, SearchControl.NONE),
                state.expectedNodes, state.expectedChecksum);
    }

    @Benchmark
    public int pittedField(PittedField state) {
        return verify(state.engine.findPath(state.request, SearchControl.NONE),
                state.expectedNodes, state.expectedChecksum);
    }

    private static int verify(PathResult result, int expectedNodes,
                              int expectedChecksum) {
        if (!result.found() || result.nodes().size() != expectedNodes
                || checksum(result) != expectedChecksum) {
            throw new IllegalStateException("incorrect result: " + result);
        }
        return checksum(result) ^ result.visitedNodes()
                ^ result.examinedNeighbors();
    }

    private static int checksum(PathResult result) {
        int checksum = 1;
        for (PathNode node : result.nodes()) {
            checksum = 31 * checksum + node.graphX();
            checksum = 31 * checksum + node.graphY();
            checksum = 31 * checksum + node.graphZ();
            checksum = 31 * checksum + node.movement().ordinal();
        }
        return checksum;
    }

    private static NavigationProfile jumping(PlatformJumpCapabilities jump) {
        return NavigationProfile.builder(NavigationMode.GROUND, MobTraversalProfile.DEFAULT, GroundCapabilities.STANDARD.withPlatformJump(jump)).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();
    }

    /** Alternating one-block islands: every edge of the route is an arc. */
    @State(Scope.Benchmark)
    public static class SteppingStones {
        final PathfindingBenchmark.World world =
                new PathfindingBenchmark.World();
        final EntityPathfinder engine = new EntityPathfinder();
        NavigationRequest request;
        int expectedNodes;
        int expectedChecksum;

        @Setup(Level.Trial)
        public void setup() {
            for (int x = 0; x <= 256; x += 2) {
                for (int z = -1; z <= 1; z++) {
                    world.set(x, 0, z, Block.STONE);
                }
            }
            request = NavigationRequest.builder(world,
                            new Pos(0.5, 1, 0.5), new Pos(128.5, 1, 0.5),
                            new BoundingBox(0.6, 1.8, 0.6),
                            jumping(PlatformJumpCapabilities.acrossGaps(1)))
                    .maxPathLength(200).nodeSearchRange(200)
                    .maxVisitedMultiplier(8).build();
            PathResult reference = engine.findPath(request, SearchControl.NONE);
            if (!reference.found()) {
                throw new IllegalStateException(
                        "stepping stones fixture is invalid: " + reference);
            }
            expectedNodes = reference.nodes().size();
            expectedChecksum = checksum(reference);
        }
    }

    /**
     * Open ground pocked with pits, so the search evaluates jump candidates
     * around every pit while walking most of the route.
     */
    @State(Scope.Benchmark)
    public static class PittedField {
        final PathfindingBenchmark.World world =
                PathfindingBenchmark.World.flat(-4, 132, -24, 24);
        final EntityPathfinder engine = new EntityPathfinder();
        NavigationRequest request;
        int expectedNodes;
        int expectedChecksum;

        @Setup(Level.Trial)
        public void setup() {
            for (int x = 4; x < 128; x += 6) {
                for (int z = -20; z <= 20; z += 5) {
                    world.set(x, 0, z, Block.AIR);
                    world.set(x + 1, 0, z, Block.AIR);
                    world.set(x, 0, z + 1, Block.AIR);
                    world.set(x + 1, 0, z + 1, Block.AIR);
                }
            }
            request = NavigationRequest.builder(world,
                            new Pos(0.5, 1, 0.5), new Pos(128.5, 1, 0.5),
                            new BoundingBox(0.6, 1.8, 0.6),
                            jumping(PlatformJumpCapabilities.acrossGaps(2)))
                    .maxPathLength(200).nodeSearchRange(200)
                    .maxVisitedMultiplier(8).build();
            PathResult reference = engine.findPath(request, SearchControl.NONE);
            if (!reference.found()) {
                throw new IllegalStateException(
                        "pitted field fixture is invalid: " + reference);
            }
            expectedNodes = reference.nodes().size();
            expectedChecksum = checksum(reference);
        }
    }
}
