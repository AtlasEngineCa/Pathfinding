package ca.atlasengine.pathfinding.adaptive;

import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.search.EntityPathfinder;
import ca.atlasengine.pathfinding.profile.GroundCapabilities;
import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.PathfindingBenchmark;
import ca.atlasengine.pathfinding.search.SearchControl;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.instance.block.Block;
import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.BenchmarkMode;
import org.openjdk.jmh.annotations.Level;
import org.openjdk.jmh.annotations.Mode;
import org.openjdk.jmh.annotations.OperationsPerInvocation;
import org.openjdk.jmh.annotations.OutputTimeUnit;
import org.openjdk.jmh.annotations.Param;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.Setup;
import org.openjdk.jmh.annotations.State;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;

/** Mass-navigation comparisons using identical validated A* routes. */
@BenchmarkMode(Mode.AverageTime)
@OutputTimeUnit(TimeUnit.NANOSECONDS)
public class SharedNavigationBenchmark {
    private static final int LOOKUPS = 1_000_000;

    @Benchmark
    public int independentAStar(CrowdState state) {
        int source = state.nextSource();
        PathResult result = state.pathfinder.findPath(
                state.requests.get(source), SearchControl.NONE);
        int checksum = checksum(result.nodes());
        if (!result.found() || checksum != state.segments.get(source).checksum) {
            throw new IllegalStateException("independent A* changed topology");
        }
        return checksum ^ result.visitedNodes();
    }

    @Benchmark
    public int routeMeshConstruction(CrowdState state) {
        SharedRouteMesh<Integer> mesh = state.buildMesh();
        if (mesh.nodeCount() != state.sourceCount + 1) {
            throw new IllegalStateException("mesh lost nodes");
        }
        return mesh.nodeCount();
    }

    @Benchmark
    public double targetFieldConstruction(CrowdState state) {
        SharedRouteMesh.TargetField<Integer> field = state.mesh.routesTo(
                state.targetKey, state.worldRevision);
        double total = 0;
        for (int source = 0; source < state.sourceCount; source++) {
            total += field.costFrom(source);
        }
        if (!Double.isFinite(total)) throw new IllegalStateException("field gap");
        return total;
    }

    @Benchmark
    @OperationsPerInvocation(LOOKUPS)
    public int cachedRouteLookupMillion(CrowdState state) {
        int checksum = 0;
        for (int lookup = 0; lookup < LOOKUPS; lookup++) {
            int source = lookup % state.sourceCount;
            SharedNavigationRoute<Integer> route = state.field
                    .routeFrom(source).orElseThrow();
            if (route != state.routes.get(source)) {
                throw new IllegalStateException("route was not shared");
            }
            checksum ^= System.identityHashCode(route);
        }
        return checksum;
    }

    @Benchmark
    @OperationsPerInvocation(LOOKUPS)
    public int cachedFollowerPlanLookupMillion(CrowdState state) {
        int checksum = 0;
        for (int lookup = 0; lookup < LOOKUPS; lookup++) {
            int source = lookup % state.sourceCount;
            NavigationPlan plan = state.routes.get(source).plan();
            if (plan != state.plans.get(source)) {
                throw new IllegalStateException("plan was not shared");
            }
            checksum ^= System.identityHashCode(plan);
        }
        return checksum;
    }

    @Benchmark
    public int followerPlanMaterialization(CrowdState state) {
        int source = state.nextSource();
        Segment segment = state.segments.get(source);
        SharedNavigationRoute<Integer> route = new SharedNavigationRoute<>(
                source, state.targetKey, List.of(segment.plan), segment.cost);
        NavigationPlan plan = route.plan();
        int checksum = checksum(plan.nodes());
        if (checksum != segment.checksum) {
            throw new IllegalStateException("materialized plan changed topology");
        }
        return checksum;
    }

    @State(Scope.Thread)
    public static class CrowdState {
        @Param({"0.8", "1.8"})
        public double width;

        final long worldRevision = 42;
        final int sourceCount = 16;
        final int targetKey = sourceCount;
        final PathfindingBenchmark.World world =
                PathfindingBenchmark.World.flat(-8, 272, -28, 28);
        final EntityPathfinder pathfinder = new EntityPathfinder();
        final List<NavigationRequest> requests = new ArrayList<>();
        final List<Segment> segments = new ArrayList<>();
        final List<SharedNavigationRoute<Integer>> routes = new ArrayList<>();
        final List<NavigationPlan> plans = new ArrayList<>();
        SharedRouteMesh<Integer> mesh;
        SharedRouteMesh.TargetField<Integer> field;
        int cursor;

        @Setup(Level.Trial)
        public void setup() {
            buildTerrain();
            BoundingBox box = new BoundingBox(width, 2.7, width);
            NavigationProfile profile = NavigationProfile.builder(NavigationMode.GROUND, MobTraversalProfile.DEFAULT, GroundCapabilities.STANDARD).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();
            Pos target = new Pos(256.5, 1, 0.5);
            for (int source = 0; source < sourceCount; source++) {
                Pos start = new Pos(source * 16 + 0.5, 1, 0.5);
                NavigationRequest request = NavigationRequest.builder(
                                world, start, target, box, profile)
                        .maxPathLength(420)
                        .nodeSearchRange(420)
                        .maxVisitedMultiplier(8)
                        .build();
                PathResult result = pathfinder.findPath(
                        request, SearchControl.NONE);
                if (!result.found()) {
                    throw new IllegalStateException(
                            "invalid crowd fixture source=" + source + ' ' + result);
                }
                NavigationPlan plan = NavigationPlan.from(request, result);
                requests.add(request);
                segments.add(new Segment(plan, graphLength(plan.nodes()),
                        checksum(plan.nodes())));
            }
            mesh = buildMesh();
            field = mesh.routesTo(targetKey, worldRevision);
            for (int source = 0; source < sourceCount; source++) {
                SharedNavigationRoute<Integer> route = field.routeFrom(source)
                        .orElseThrow();
                NavigationPlan plan = route.plan();
                if (checksum(plan.nodes()) != segments.get(source).checksum) {
                    throw new IllegalStateException("shared route is not equivalent");
                }
                routes.add(route);
                plans.add(plan);
            }
        }

        SharedRouteMesh<Integer> buildMesh() {
            SharedRouteMesh.Builder<Integer> builder =
                    SharedRouteMesh.builder(worldRevision);
            builder.node(targetKey);
            for (int source = 0; source < sourceCount; source++) {
                Segment segment = segments.get(source);
                builder.route(source, targetKey, segment.plan, segment.cost);
            }
            return builder.build();
        }

        int nextSource() {
            int value = cursor++;
            if (cursor == sourceCount) cursor = 0;
            return value;
        }

        private void buildTerrain() {
            boolean positive = true;
            for (int x = 24; x < 248; x += 32) {
                int gap = positive ? 15 : -15;
                for (int z = -22; z <= 22; z++) {
                    if (Math.abs(z - gap) > 3) {
                        world.column(x, z, 1, 4, Block.STONE);
                    }
                }
                positive = !positive;
            }
            // Repeated broad terraces exercise large-footprint ascent and
            // descent without introducing benchmark-only unsupported cells.
            for (int base = 8; base < 248; base += 48) {
                for (int x = base; x < base + 8; x++) {
                    for (int z = -6; z <= 6; z++) {
                        world.set(x, 1, z, Block.STONE);
                    }
                }
            }
        }
    }

    private static int checksum(List<PathNode> nodes) {
        int checksum = 1;
        for (PathNode node : nodes) {
            checksum = 31 * checksum + node.graphX();
            checksum = 31 * checksum + node.graphY();
            checksum = 31 * checksum + node.graphZ();
            checksum = 31 * checksum + node.movement().ordinal();
        }
        return checksum;
    }

    private static double graphLength(List<PathNode> nodes) {
        double length = 0;
        for (int index = 1; index < nodes.size(); index++) {
            PathNode first = nodes.get(index - 1);
            PathNode second = nodes.get(index);
            double dx = second.graphX() - first.graphX();
            double dy = second.graphY() - first.graphY();
            double dz = second.graphZ() - first.graphZ();
            length += Math.sqrt(dx * dx + dy * dy + dz * dz);
        }
        return length;
    }

    private record Segment(
            NavigationPlan plan, double cost, int checksum) {
    }
}
