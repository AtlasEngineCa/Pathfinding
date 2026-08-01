package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.Locale;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Human-readable route samples used to audit actual output, not just status.
 * Gradle records stdout in the XML test report for manual inspection.
 */
class PathOutputAuditTest {
    private static final BoundingBox MOB = new BoundingBox(0.6, 1.8, 0.6);

    @Test
    void emitRepresentativeRoutesAndVerifyTheirGeometry() {
        TestWorld flat = new TestWorld().floor(-15, 15, -15, 15, 0, Block.STONE);
        PathResult straight = new DiscreteGroundPathfinder().findPath(flat, new Pos(0.5, 1, 0.5), new Pos(10.5, 1, 0.5), MOB, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(40).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
        emit("straight", straight);
        assertEquals(11, straight.nodes().size());
        for (int i = 0; i <= 10; i++) {
            assertEquals(i, straight.nodes().get(i).blockX());
            assertEquals(0, straight.nodes().get(i).blockZ());
        }
        assertEquals(10, geometricLength(straight), 1.0e-9);

        TestWorld wall = new TestWorld().floor(-15, 15, -15, 15, 0, Block.STONE);
        for (int x = -2; x <= 2; x++) wall.column(x, 4, 1, 2, Block.STONE);
        PathResult detour = new DiscreteGroundPathfinder().findPath(wall, new Pos(0.5, 1, 0.5), new Pos(0.5, 1, 8.5), MOB, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(40).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
        emit("wall-detour", detour);
        assertTrue(detour.found());
        assertEquals(8, detour.nodes().size() - 1,
                "five-wide wall has an eight-edge shortest grid detour");
        assertEquals(6 * Math.sqrt(2) + 2, geometricLength(detour), 1.0e-9);
        assertTrue(detour.nodes().stream().anyMatch(node -> Math.abs(node.blockX()) == 3));

        TestWorld hazard = new TestWorld().floor(-10, 10, -10, 10, 0, Block.STONE)
                .set(0, 1, 3, Block.CACTUS);
        PathResult cactus = new DiscreteGroundPathfinder().findPath(hazard, new Pos(0.5, 1, 0.5), new Pos(0.5, 1, 7.5), MOB, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(40).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
        emit("cactus-detour", cactus);
        assertTrue(cactus.nodes().stream().noneMatch(node ->
                node.blockX() == 0 && node.blockZ() == 3));
        assertEquals(4 * Math.sqrt(2) + 3, geometricLength(cactus), 1.0e-9,
                "DAMAGING_IN_NEIGHBOR makes the least-cost route keep one-cell clearance");

        TestWorld wetLane = new TestWorld().floor(-3, 8, -3, 3, 0, Block.STONE);
        for (int x = 1; x <= 4; x++) wetLane.set(x, 1, 0, Block.WATER);
        PathResult waterAvoiding = groundPath(wetLane, MobTraversalProfile.DEFAULT);
        PathResult waterPreferring = groundPath(
                wetLane, MobTraversalProfile.CHICKEN);
        emit("default-water", waterAvoiding);
        emit("floating-water-walker", waterPreferring);
        // WATER itself remains costly, while its dry border is free. The
        // least-cost route therefore follows the one-cell shoreline instead
        // of making a wide detour around the complete border band.
        assertEquals(3 + 2 * Math.sqrt(2),
                geometricLength(waterAvoiding), 1.0e-9);
        assertEquals(5, integerGraphLength(waterPreferring), 1.0e-9,
                "baseline A* charges five integer graph edges");
        assertEquals(3 + 2 * Math.sqrt(1.25),
                geometricLength(waterPreferring), 1.0e-9,
                "movement projection raises floating water nodes by half a block");
        assertTrue(waterAvoiding.nodes().stream().anyMatch(node -> node.blockZ() != 0));
        assertTrue(waterPreferring.nodes().stream().allMatch(node -> node.blockZ() == 0));

        TestWorld lavaLane = new TestWorld().floor(-3, 8, -3, 3, 0, Block.STONE)
                .set(2, 1, 0, Block.LAVA);
        PathResult lavaAvoiding = groundPath(lavaLane, MobTraversalProfile.DEFAULT);
        PathResult lavaPreferring = groundPath(lavaLane, MobTraversalProfile.STRIDER);
        emit("default-lava", lavaAvoiding);
        emit("strider-lava", lavaPreferring);
        assertEquals(3 * Math.sqrt(2) + 3, geometricLength(lavaAvoiding), 1.0e-9,
                "FIRE_IN_NEIGHBOR expands the avoided region around lava");
        assertEquals(5, geometricLength(lavaPreferring), 1.0e-9);

        TestWorld fence = new TestWorld().floor(-3, 8, -2, 2, 0, Block.STONE);
        for (int x = -2; x <= 8; x++) {
            fence.column(x, -1, 1, 4, Block.STONE);
            fence.column(x, 1, 1, 4, Block.STONE);
        }
        fence.set(2, 1, 0, Block.OAK_FENCE);
        PathResult camel = new EntityPathfinder().findPath(
                NavigationRequest.builder(
                                fence, new Pos(0.5, 1, 0.5),
                                new Pos(5.5, 1, 0.5),
                                new BoundingBox(0.9, 1.8, 0.9),
                                BuiltinNavigationProfiles.forEntityType(
                                        EntityType.CAMEL))
                        .maxPathLength(20)
                        .maxVisitedMultiplier(8)
                        .build(), SearchControl.NONE);
        emit("camel-fence", camel);
        assertTrue(camel.found());
        assertEquals(2.5, camel.nodes().stream().mapToDouble(PathNode::y)
                .max().orElseThrow(), 1.0e-4);

        TestWorld water = new TestWorld();
        for (int x = -1; x <= 5; x++) {
            for (int y = -1; y <= 3; y++) {
                for (int z = -2; z <= 2; z++) water.set(x, y, z, Block.WATER);
            }
        }
        PathResult swim = new EntityPathfinder().findPath(
                NavigationRequest.builder(
                                water, new Vec(0.5, 0.5, 0.5),
                                new Vec(4.5, 2.5, 0.5),
                                new BoundingBox(0.6, 0.6, 0.6),
                                BuiltinNavigationProfiles.forEntityType(
                                        EntityType.COD))
                        .maxPathLength(20)
                        .maxVisitedMultiplier(4)
                        .build(), SearchControl.NONE);
        emit("cod-3d", swim);
        assertTrue(swim.found());
        assertEquals(5, geometricLength(swim), 1.0e-9,
                "start floor(y+0.5) and target floor(y) leave four horizontal "
                        + "and one vertical baseline swim edges");
    }

    private static void emit(String label, PathResult result) {
        System.out.printf(Locale.ROOT,
                "AUDIT %-15s status=%s visited=%d examined=%d length=%.6f nodes=%s%n",
                label, result.status(), result.visitedNodes(),
                result.examinedNeighbors(), geometricLength(result), result.nodes());
    }

    private static PathResult groundPath(TestWorld world, MobTraversalProfile profile) {
        return new DiscreteGroundPathfinder().findPath(world, new Pos(0.5, 1, 0.5), new Pos(5.5, 1, 0.5), MOB, profile, GroundSearchLimits.builder().maxPathLength(20).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
    }

    private static double geometricLength(PathResult result) {
        double length = 0;
        for (int i = 1; i < result.nodes().size(); i++) {
            length += result.nodes().get(i - 1).asVec()
                    .distance(result.nodes().get(i).asVec());
        }
        return length;
    }

    private static double integerGraphLength(PathResult result) {
        double length = 0;
        for (int i = 1; i < result.nodes().size(); i++) {
            PathNode previous = result.nodes().get(i - 1);
            PathNode node = result.nodes().get(i);
            int dx = node.blockX() - previous.blockX();
            int dy = node.blockY() - previous.blockY();
            int dz = node.blockZ() - previous.blockZ();
            length += Math.sqrt(dx * dx + dy * dy + dz * dz);
        }
        return length;
    }
}
