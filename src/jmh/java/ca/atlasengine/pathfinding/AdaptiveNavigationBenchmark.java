package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.adaptive.SharedMeshPolicy;
import ca.atlasengine.pathfinding.internal.adaptive.SharedMeshCoordinator;
import ca.atlasengine.pathfinding.adaptive.SharedMeshHandle;
import ca.atlasengine.pathfinding.adaptive.SharedMeshSource;
import ca.atlasengine.pathfinding.NavigationPlan;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.BenchmarkMode;
import org.openjdk.jmh.annotations.Level;
import org.openjdk.jmh.annotations.Mode;
import org.openjdk.jmh.annotations.OutputTimeUnit;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.Setup;
import org.openjdk.jmh.annotations.State;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

/** Measures coordinator overhead separately from the A* work it delegates. */
@BenchmarkMode(Mode.AverageTime)
@OutputTimeUnit(TimeUnit.NANOSECONDS)
public class AdaptiveNavigationBenchmark {
    @Benchmark
    public int loneMobObservation(AdaptiveState state) {
        SharedMeshHandle handle = state.unpromoted.request(
                "lone", "target", 1, state.tick++, state.request);
        NavigationPlan plan = handle.plan().join();
        handle.close();
        if (handle.source() != SharedMeshSource.INDIVIDUAL_SEARCH
                || plan.nodes().size() != state.plan.nodes().size()) {
            throw new IllegalStateException("lone request changed");
        }
        return plan.nodes().size();
    }

    @Benchmark
    public int promotedCorridorRequest(AdaptiveState state) {
        SharedMeshHandle handle = state.promoted.request(
                "shared", "target", 1, state.tick++, state.request);
        NavigationPlan plan = handle.plan().join();
        handle.close();
        if (handle.source() != SharedMeshSource.SHARED_TARGET_FIELD
                || plan.nodes().size() != state.plan.nodes().size()) {
            throw new IllegalStateException("shared request changed");
        }
        return plan.nodes().size();
    }

    @State(Scope.Thread)
    public static class AdaptiveState {
        final PathfindingBenchmark.World world =
                PathfindingBenchmark.World.flat(-2, 260, -2, 2);
        final NavigationProfile profile =
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
        final BoundingBox box = new BoundingBox(0.8, 1.8, 0.8);
        final NavigationRequest request = NavigationRequest.builder(
                        world, new Pos(0.5, 1, 0.5),
                        new Pos(256.5, 1, 0.5), box, profile)
                .maxPathLength(320).nodeSearchRange(320).build();
        final NavigationPlan plan = straightPlan();
        SharedMeshCoordinator unpromoted;
        SharedMeshCoordinator promoted;
        long tick;

        @Setup(Level.Trial)
        public void setup() {
            SharedMeshPolicy cold = SharedMeshPolicy.builder()
                    .promotionRequests(Integer.MAX_VALUE)
                    .retentionRequests(1).maximumNodesPerRegion(1_024).build();
            SharedMeshPolicy hot = SharedMeshPolicy.builder()
                    .promotionRequests(2).retentionRequests(1)
                    .maximumNodesPerRegion(1_024).build();
            unpromoted = new SharedMeshCoordinator(
                    (actor, ignored) -> CompletableFuture.completedFuture(plan), cold);
            promoted = new SharedMeshCoordinator(
                    (actor, ignored) -> CompletableFuture.completedFuture(plan), hot);
            promoted.request("seed-1", "target", 1, 0, request).plan().join();
            promoted.request("seed-2", "target", 1, 1, request).plan().join();
            if (promoted.stats().promotedRegions() != 1) {
                throw new IllegalStateException("benchmark failed to promote");
            }
        }

        private NavigationPlan straightPlan() {
            List<PathNode> nodes = new ArrayList<>();
            for (int x = 0; x <= 256; x++) {
                nodes.add(new PathNode(x + 0.5, 1, 0.5,
                        PathNode.Movement.WALK, x, 1, 0));
            }
            return new NavigationPlan(
                    request.start(), request.target(), box, profile,
                    PathStatus.FOUND, nodes, nodes.size(), nodes.size() * 8);
        }
    }
}
