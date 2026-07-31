package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
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

import static java.lang.Math.abs;

/**
 * Reusable long-route performance benchmarks with correctness guards.
 *
 * <p>Each invocation constructs fresh search-local state but reuses an
 * immutable world fixture, matching normal server usage. Every result is
 * checked for status, endpoint, path validity, and a stable topology checksum
 * so performance regressions cannot be hidden by incorrect early exits.</p>
 */
@BenchmarkMode(Mode.AverageTime)
@OutputTimeUnit(TimeUnit.MICROSECONDS)
public class PathfindingBenchmark {
    @Benchmark
    public int openGround512(OpenGround state) {
        PathResult result = state.search();
        return verify(result, state.world, state.targetVector, state.box, 513,
                state.expectedChecksum);
    }

    @Benchmark
    public int alternatingWalls256(AlternatingWalls state) {
        PathResult result = state.search();
        return verify(result, state.world, state.target, state.box,
                state.expectedNodes, state.expectedChecksum);
    }

    @Benchmark
    public int largeMobUneven256(LargeUneven state) {
        PathResult result = state.engine.findPath(
                state.request, SearchControl.NONE);
        return verify(result, state.world, state.target, state.box,
                state.expectedNodes, state.expectedChecksum);
    }

    @Benchmark
    public int flyingVolume128(FlyingVolume state) {
        PathResult result = state.engine.findPath(
                state.request, SearchControl.NONE);
        return verify(result, state.world, state.target,
                state.request.boundingBox(),
                state.expectedNodes, state.expectedChecksum);
    }

    @Benchmark
    public int climbableTower32(ClimbableTower state) {
        PathResult result = state.engine.findPath(
                state.request, SearchControl.NONE);
        if (!result.found() || result.nodes().size() != state.expectedNodes
                || checksum(result) != state.expectedChecksum
                || result.nodes().stream().noneMatch(node ->
                node.movement() == PathNode.Movement.CLIMB)) {
            throw new IllegalStateException("incorrect climb result: " + result);
        }
        return result.visitedNodes() ^ result.examinedNeighbors()
                ^ result.nodes().size();
    }

    private static int verify(
            PathResult result, World world, Vec target,
            BoundingBox box,
            int expectedNodes, int expectedChecksum) {
        if (!result.found() || result.nodes().size() != expectedNodes) {
            throw new IllegalStateException("incorrect result: " + result);
        }
        PathNode end = result.nodes().getLast();
        if (end.asVec().distance(target) > 1.5) {
            throw new IllegalStateException("incorrect endpoint: " + end);
        }
        int checksum = checksum(result);
        if (checksum != expectedChecksum) {
            throw new IllegalStateException("topology changed: " + checksum
                    + " expected=" + expectedChecksum);
        }
        double halfWidth = box.width() * 0.5;
        double halfDepth = box.depth() * 0.5;
        for (PathNode node : result.nodes()) {
            if (node.movement() != PathNode.Movement.FLY
                    && node.movement() != PathNode.Movement.SWIM) {
                int minX = (int) Math.floor(node.x() - halfWidth + 1.0e-5);
                int maxX = (int) Math.floor(node.x() + halfWidth - 1.0e-5);
                int minZ = (int) Math.floor(node.z() - halfDepth + 1.0e-5);
                int maxZ = (int) Math.floor(node.z() + halfDepth - 1.0e-5);
                boolean supported = false;
                for (int x = minX; x <= maxX && !supported; x++) {
                    for (int z = minZ; z <= maxZ; z++) {
                        if (!world.get(x, node.graphY() - 1, z).air()) {
                            supported = true;
                            break;
                        }
                    }
                }
                if (!supported) {
                    throw new IllegalStateException("unsupported node: " + node);
                }
            }
        }
        return checksum ^ result.visitedNodes() ^ result.examinedNeighbors();
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

    @State(Scope.Benchmark)
    public static class OpenGround {
        final World world = World.flat(-4, 516, -4, 4);
        final DiscreteGroundPathfinder engine = new DiscreteGroundPathfinder();
        final Pos start = new Pos(0.5, 1, 0.5);
        final Pos target = new Pos(512.5, 1, 0.5);
        final Vec targetVector = target.asVec();
        final BoundingBox box = new BoundingBox(0.8, 1.8, 0.8);
        int expectedChecksum;

        PathResult search() {
            return engine.findPath(world, start, target, box, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(520).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
        }

        @Setup(Level.Trial)
        public void setup() {
            PathResult reference = search();
            requireFound(reference, "open ground");
            if (reference.nodes().size() != 513) {
                throw new IllegalStateException("open-ground topology changed");
            }
            expectedChecksum = checksum(reference);
        }
    }

    @State(Scope.Benchmark)
    public static class AlternatingWalls {
        final World world = World.flat(-4, 260, -24, 24);
        final DiscreteGroundPathfinder engine = new DiscreteGroundPathfinder();
        final Pos start = new Pos(0.5, 1, 0.5);
        final Pos targetPosition = new Pos(256.5, 1, 0.5);
        final Vec target = targetPosition.asVec();
        final BoundingBox box = new BoundingBox(0.8, 1.8, 0.8);
        int expectedNodes;
        int expectedChecksum;

        PathResult search() {
            return engine.findPath(world, start, targetPosition, box, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(400).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
        }

        @Setup(Level.Trial)
        public void setup() {
            boolean upper = true;
            for (int x = 16; x < 256; x += 16) {
                int gap = upper ? 14 : -14;
                for (int z = -20; z <= 20; z++) {
                    if (abs(z - gap) > 1) world.column(x, z, 1, 3, Block.STONE);
                }
                upper = !upper;
            }
            PathResult reference = search();
            requireFound(reference, "alternating walls");
            expectedNodes = reference.nodes().size();
            expectedChecksum = checksum(reference);
        }
    }

    @State(Scope.Benchmark)
    public static class LargeUneven {
        final World world = World.flat(-4, 260, -8, 8);
        final EntityPathfinder engine = new EntityPathfinder();
        final Pos start = new Pos(0.5, 1, 0.5);
        final Pos targetPosition = new Pos(256.5, 1, 0.5);
        final Vec target = targetPosition.asVec();
        final BoundingBox box = new BoundingBox(1.8, 2.7, 1.8);
        NavigationRequest request;
        int expectedNodes;
        int expectedChecksum;

        @Setup(Level.Trial)
        public void setup() {
            for (int base = 12; base < 244; base += 24) {
                for (int x = base; x < base + 8; x++) {
                    for (int z = -3; z <= 3; z++) world.set(x, 1, z, Block.STONE);
                }
                for (int x = base + 8; x < base + 16; x++) {
                    for (int z = -3; z <= 3; z++) {
                        world.set(x, 1, z, Block.STONE);
                        world.set(x, 2, z, Block.STONE);
                    }
                }
            }
            NavigationProfile profile = NavigationProfile.builder(NavigationMode.GROUND, MobTraversalProfile.DEFAULT, GroundCapabilities.STANDARD).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();
            request = NavigationRequest.builder(
                            world, start, targetPosition, box, profile)
                    .maxPathLength(400)
                    .nodeSearchRange(400)
                    .maxVisitedMultiplier(8)
                    .build();
            PathResult reference = engine.findPath(request, SearchControl.NONE);
            requireFound(reference, "large uneven");
            expectedNodes = reference.nodes().size();
            expectedChecksum = checksum(reference);
        }
    }

    @State(Scope.Benchmark)
    public static class FlyingVolume {
        final World world = new World();
        final EntityPathfinder engine = new EntityPathfinder();
        final Vec target = new Vec(128.5, 0.5, 0.5);
        NavigationRequest request;
        int expectedNodes;
        int expectedChecksum;

        @Setup(Level.Trial)
        public void setup() {
            for (int x = 16; x < 128; x += 16) {
                for (int y = -3; y <= 3; y++) {
                    for (int z = -3; z <= 3; z++) world.set(x, y, z, Block.STONE);
                }
            }
            NavigationProfile profile = NavigationProfile.builder(NavigationMode.FLYING, MobTraversalProfile.DEFAULT, GroundCapabilities.STANDARD).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();
            request = NavigationRequest.builder(world,
                            new Vec(0.5, 0.5, 0.5), target,
                            new BoundingBox(0.6, 0.6, 0.6), profile)
                    .maxPathLength(180)
                    .maxVisitedMultiplier(8)
                    .build();
            PathResult reference = engine.findPath(request, SearchControl.NONE);
            requireFound(reference, "flying volume");
            expectedNodes = reference.nodes().size();
            expectedChecksum = checksum(reference);
        }
    }

    @State(Scope.Benchmark)
    public static class ClimbableTower {
        final World world = World.flat(-4, 6, -2, 2);
        final EntityPathfinder engine = new EntityPathfinder();
        final NavigationRequest request;
        int expectedNodes;
        int expectedChecksum;

        public ClimbableTower() {
            Block ladder = Block.LADDER.withProperty("facing", "west");
            for (int y = 1; y <= 32; y++) {
                world.set(1, y, 0, ladder);
                world.set(2, y, 0, Block.STONE);
            }
            NavigationProfile base = BuiltinNavigationProfiles
                    .forEntityType(
                    net.minestom.server.entity.EntityType.ZOMBIE);
            NavigationProfile climbing = NavigationProfile.builder(base.mode(), base.mobProfile(), base.groundCapabilities().withClimbables(
                            ClimbableCapabilities.STANDARD)).allowBreaching(base.allowBreaching()).prefersShallowWater(base.prefersShallowWater()).avoidSun(base.avoidSun()).directPathIgnoresFluids(base.directPathIgnoresFluids()).pathToTargetsBelowSurface(base.pathToTargetsBelowSurface()).build();
            request = NavigationRequest.builder(world,
                            new Pos(0.5, 1, 0.5),
                            new Pos(1.5, 33, 0.5),
                            new BoundingBox(0.6, 1.8, 0.6), climbing)
                    .maxPathLength(64).nodeSearchRange(64).build();
        }

        @Setup(Level.Trial)
        public void setup() {
            PathResult reference = engine.findPath(
                    request, SearchControl.NONE);
            requireFound(reference, "climbable tower");
            expectedNodes = reference.nodes().size();
            expectedChecksum = checksum(reference);
        }
    }

    private static void requireFound(PathResult result, String fixture) {
        if (!result.found()) {
            throw new IllegalStateException(fixture + " fixture is invalid: " + result);
        }
    }

    public static final class World implements Block.Getter {
        private static final int MIN_X = -32;
        private static final int MIN_Y = -16;
        private static final int MIN_Z = -64;
        private static final int SIZE_X = 704;
        private static final int SIZE_Y = 64;
        private static final int SIZE_Z = 128;
        private final Block[] blocks = new Block[SIZE_X * SIZE_Y * SIZE_Z];

        public static World flat(int minX, int maxX, int minZ, int maxZ) {
            World world = new World();
            for (int x = minX; x <= maxX; x++) {
                for (int z = minZ; z <= maxZ; z++) {
                    world.set(x, 0, z, Block.STONE);
                }
            }
            return world;
        }

        public World set(int x, int y, int z, Block block) {
            blocks[index(x, y, z)] = block == Block.AIR ? null : block;
            return this;
        }

        public World column(int x, int z, int minY, int maxY, Block block) {
            for (int y = minY; y <= maxY; y++) set(x, y, z, block);
            return this;
        }

        Block get(int x, int y, int z) {
            if (x < MIN_X || x >= MIN_X + SIZE_X
                    || y < MIN_Y || y >= MIN_Y + SIZE_Y
                    || z < MIN_Z || z >= MIN_Z + SIZE_Z) return Block.AIR;
            Block block = blocks[index(x, y, z)];
            return block == null ? Block.AIR : block;
        }

        @Override
        public Block getBlock(int x, int y, int z, Condition condition) {
            return get(x, y, z);
        }

        private static int index(int x, int y, int z) {
            return ((x - MIN_X) * SIZE_Y + y - MIN_Y) * SIZE_Z
                    + z - MIN_Z;
        }
    }
}
