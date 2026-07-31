package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import jdk.jfr.Configuration;
import jdk.jfr.Recording;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;

import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;

/** Opt-in JFR workload used by the {@code profilePathfinding} Gradle task. */
public final class PathfindingProfileMain {
    private static final ThreadMXBean THREADS =
            ManagementFactory.getThreadMXBean();
    private static volatile int resultSink;

    private PathfindingProfileMain() {
    }

    public static void main(String[] arguments) throws Exception {
        Path output = Path.of(arguments[0]);
        Files.createDirectories(output.getParent());

        Workload ground = groundWorkload();
        Workload maze = mazeWorkload();
        Workload flying = flyingWorkload();
        Workload large = largeBoxWorkload();
        Workload climbing = climbableWorkload();
        boolean climbingOnly = arguments.length > 1
                && arguments[1].equals("climbing");
        for (int index = 0; index < 30; index++) {
            if (!climbingOnly) {
                ground.run();
                maze.run();
                flying.run();
                large.run();
            }
            climbing.run();
        }

        try (Recording recording = new Recording(
                Configuration.getConfiguration("profile"))) {
            recording.setName("minestom-pathfinding-profile");
            recording.start();
            if (climbingOnly) {
                measure("climbable-tower32", climbing, 5_000);
            } else {
                measure("ground-obstacles", ground, 1_000);
                measure("hard-maze", maze, 80);
                measure("flying-volume", flying, 1_000);
                measure("large-box-terrain", large, 500);
                measure("climbable-tower32", climbing, 1_000);
            }
            recording.stop();
            recording.dump(output);
        }
        System.out.println("PROFILE recording=" + output.toAbsolutePath()
                + " sink=" + resultSink);
    }

    private static void measure(String name, Workload workload, int iterations) {
        long cpuStart = THREADS.isCurrentThreadCpuTimeSupported()
                ? THREADS.getCurrentThreadCpuTime() : 0;
        long wallStart = System.nanoTime();
        long visited = 0;
        long examined = 0;
        for (int index = 0; index < iterations; index++) {
            PathResult result = workload.run();
            if (!result.found()) throw new IllegalStateException(name + ": " + result);
            visited += result.visitedNodes();
            examined += result.examinedNeighbors();
            resultSink ^= result.nodes().size();
        }
        long wall = System.nanoTime() - wallStart;
        long cpu = THREADS.isCurrentThreadCpuTimeSupported()
                ? THREADS.getCurrentThreadCpuTime() - cpuStart : 0;
        System.out.printf(Locale.ROOT,
                "PROFILE %-18s iterations=%d wall_ms=%.3f cpu_ms=%.3f "
                        + "wall_us_op=%.3f cpu_us_op=%.3f visited_op=%.1f examined_op=%.1f%n",
                name, iterations, wall / 1e6, cpu / 1e6,
                wall / 1e3 / iterations, cpu / 1e3 / iterations,
                (double) visited / iterations, (double) examined / iterations);
    }

    private static Workload groundWorkload() {
        TestWorld world = flat(50);
        for (int z = -12; z <= 12; z++) {
            if (z < 8 || z > 10) world.column(12, z, 1, 3, Block.STONE);
            if (z < -10 || z > -8) world.column(24, z, 1, 3, Block.STONE);
            if (z < 6 || z > 8) world.column(36, z, 1, 3, Block.STONE);
        }
        DiscreteGroundPathfinder engine = new DiscreteGroundPathfinder();
        return () -> engine.findPath(world, new Pos(0.5, 1, 0.5), new Pos(45.5, 1, 0.5), new BoundingBox(0.8, 1.8, 0.8), MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(100).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
    }

    private static Workload mazeWorkload() {
        TestWorld world = flat(32);
        boolean positive = true;
        for (int z = -16; z <= 16; z += 4) {
            int gap = positive ? 13 : -13;
            for (int x = -17; x <= 17; x++) {
                if (Math.abs(x - gap) > 1) world.column(x, z, 1, 3, Block.STONE);
            }
            positive = !positive;
        }
        DiscreteGroundPathfinder engine = new DiscreteGroundPathfinder();
        return () -> engine.findPath(world, new Pos(0.5, 1, -20.5), new Pos(0.5, 1, 20.5), new BoundingBox(0.8, 1.8, 0.8), MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(200).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
    }

    private static Workload flyingWorkload() {
        TestWorld world = new TestWorld();
        for (int x = 8; x <= 32; x += 8) {
            for (int y = -3; y <= 3; y++) {
                for (int z = -3; z <= 3; z++) world.set(x, y, z, Block.STONE);
            }
        }
        EntityPathfinder engine = new EntityPathfinder();
        NavigationProfile profile = NavigationProfile.builder(NavigationMode.FLYING, MobTraversalProfile.DEFAULT, GroundCapabilities.STANDARD).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();
        NavigationRequest request = NavigationRequest.builder(world,
                        new Vec(0.5, 0.5, 0.5), new Vec(40.5, 0.5, 0.5),
                        new BoundingBox(0.6, 0.6, 0.6), profile)
                .maxPathLength(80)
                .build();
        return () -> engine.findPath(request, SearchControl.NONE);
    }

    private static Workload largeBoxWorkload() {
        TestWorld world = flat(50);
        for (int x = 8; x <= 36; x += 7) {
            for (int z = -3; z <= 3; z++) {
                if (z != 3) world.column(x, z, 1, 3, Block.STONE);
            }
        }
        EntityPathfinder engine = new EntityPathfinder();
        NavigationProfile profile = NavigationProfile.builder(NavigationMode.GROUND, MobTraversalProfile.DEFAULT, GroundCapabilities.STANDARD).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();
        NavigationRequest request = NavigationRequest.builder(world,
                        new Pos(0.5, 1, 0.5), new Pos(42.5, 1, 0.5),
                        new BoundingBox(1.8, 2.7, 1.8), profile)
                .maxPathLength(100)
                .maxVisitedMultiplier(8)
                .build();
        return () -> engine.findPath(request, SearchControl.NONE);
    }

    private static Workload climbableWorkload() {
        TestWorld world = flat(6);
        Block ladder = Block.LADDER.withProperty("facing", "west");
        for (int y = 1; y <= 32; y++) {
            world.set(1, y, 0, ladder);
            world.set(2, y, 0, Block.STONE);
        }
        EntityPathfinder engine = new EntityPathfinder();
        NavigationProfile base = BuiltinNavigationProfiles.forEntityType(
                net.minestom.server.entity.EntityType.ZOMBIE);
        NavigationProfile climbing = NavigationProfile.builder(base.mode(), base.mobProfile(), base.groundCapabilities().withClimbables(
                        ClimbableCapabilities.STANDARD)).allowBreaching(base.allowBreaching()).prefersShallowWater(base.prefersShallowWater()).avoidSun(base.avoidSun()).directPathIgnoresFluids(base.directPathIgnoresFluids()).pathToTargetsBelowSurface(base.pathToTargetsBelowSurface()).build();
        NavigationRequest request = NavigationRequest.builder(world,
                        new Pos(0.5, 1, 0.5), new Pos(1.5, 33, 0.5),
                        new BoundingBox(0.6, 1.8, 0.6), climbing)
                .maxPathLength(64)
                .nodeSearchRange(64)
                .build();
        return () -> engine.findPath(request, SearchControl.NONE);
    }

    private static TestWorld flat(int radius) {
        return new TestWorld().floor(-radius, radius, -radius, radius, 0, Block.STONE);
    }

    @FunctionalInterface
    private interface Workload {
        PathResult run();
    }
}
