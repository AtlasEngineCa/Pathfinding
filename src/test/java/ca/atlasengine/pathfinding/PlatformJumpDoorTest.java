package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.internal.JumpArc;
import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.OpenableBlockFamily;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Platform jumps around blocks a mob may open. The rule is deliberately
 * one-sided: the follower opens the landing waypoint's own column as the
 * first act of the jump, so the search reads that column in its opened
 * states and every other cell of the arc exactly as it stands.
 */
class PlatformJumpDoorTest {
    private static final BoundingBox NARROW = new BoundingBox(0.6, 1.8, 0.6);
    private static final Block CLOSED_GATE = Block.OAK_FENCE_GATE
            .withProperty("facing", "north")
            .withProperty("open", "false");
    private static final Block CLOSED_DOOR = Block.OAK_DOOR
            .withProperty("facing", "east")
            .withProperty("hinge", "left")
            .withProperty("half", "lower")
            .withProperty("open", "false");
    private static final Block CLOSED_TRAPDOOR = Block.OAK_TRAPDOOR
            .withProperty("facing", "east")
            .withProperty("half", "bottom")
            .withProperty("open", "false");
    private static final MobTraversalProfile OPENER = MobTraversalProfile
            .builder("opener")
            .blockManipulation(BlockManipulationCapabilities.of(
                    OpenableBlockFamily.values()))
            .build();
    private static final MobTraversalProfile DOORS_ONLY = MobTraversalProfile
            .builder("doors_only").canOpenDoors(true).build();

    @Test
    void aLandingIsOpenableOnlyForAProfileThatOpensThatFamily() {
        PathResult gateForOpener = jump(landing(CLOSED_GATE), OPENER);
        PathResult gateForDoors = jump(landing(CLOSED_GATE), DOORS_ONLY);
        PathResult doorForDoors = jump(landing(CLOSED_DOOR), DOORS_ONLY);
        PathResult doorForPlain = jump(
                landing(CLOSED_DOOR), MobTraversalProfile.DEFAULT);

        assertLanded(gateForOpener);
        assertLanded(doorForDoors);
        assertEquals(PathStatus.PARTIAL, gateForDoors.status(),
                () -> "a fence gate is not a door: " + gateForDoors.nodes());
        assertEquals(PathStatus.PARTIAL, doorForPlain.status(),
                () -> "a profile that opens nothing may not land in a door: "
                        + doorForPlain.nodes());
    }

    /**
     * The column is read in the states the follower opens it into, not as
     * air. An open trapdoor is a panel across the face it swung to, and the
     * one facing the approach is struck by the very arc that would land
     * there.
     */
    @ParameterizedTest
    @CsvSource({"east, false", "west, true", "north, true", "south, true"})
    void anOpenedLandingIsSweptAsAShapeRatherThanAsAir(
            String facing, boolean jumpable) {
        Block trapdoor = CLOSED_TRAPDOOR.withProperty("facing", facing);

        PathResult result = jump(landing(trapdoor), OPENER);

        assertEquals(jumpable, result.found(), () -> "facing=" + facing
                + " landed " + result.nodes());
        assertFalse(jump(landing(trapdoor), MobTraversalProfile.DEFAULT)
                .found(), "a plain profile never lands in a trapdoor");
    }

    /**
     * The stated limit. Nothing opens a block the arc merely passes over, so
     * reading one as open would emit an edge the launch refuses every time,
     * and the entity would stand on the takeoff until it is declared stuck.
     * The identical fixture with the gate already open is jumped, so the
     * refusal is the shut gate and nothing else about the geometry.
     */
    @ParameterizedTest
    @ValueSource(booleans = {false, true})
    void anArcOverAnOpenableBlockIsRefusedUntilThatBlockIsOpen(boolean open) {
        TestWorld world = new TestWorld()
                .set(0, 0, 0, Block.STONE)
                .set(4, 0, 0, Block.STONE)
                .set(2, 0, 0, Block.STONE)
                .set(2, 1, 0, Block.STONE)
                .set(2, 2, 0, CLOSED_GATE.withProperty(
                        "open", String.valueOf(open)));
        PlatformJumpCapabilities jump =
                PlatformJumpCapabilities.builder().maxHorizontalDistance(4).maxRise(0).maxDrop(0).apexClearance(1.5).build();
        Pos start = new Pos(0.5, 1, 0.5);
        Pos goal = new Pos(4.5, 1, 0.5);

        PathResult result = new DiscreteGroundPathfinder().findPath(world, start, goal, NARROW, OPENER, GroundSearchLimits.builder().maxPathLength(48).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD.withPlatformJump(jump), SearchControl.NONE);

        assertEquals(open, result.found(), result.nodes()::toString);
        assertEquals(open, JumpArc.clear(world, NARROW, start, goal, 1.5,
                        null),
                "the search and the launch have to read the wall alike");
    }

    @Test
    void aTakeoffStandingInAnOpenableBlockNeverLaunches() {
        TestWorld world = new TestWorld()
                .set(0, 0, 0, Block.STONE)
                .set(0, 1, 0, CLOSED_GATE)
                .set(3, 0, 0, Block.STONE);

        PathResult result = jump(world, OPENER);

        assertTrue(result.nodes().stream().noneMatch(node ->
                        node.movement() == PathNode.Movement.JUMP),
                () -> "the follower walks onto its takeoff anchor before it "
                        + "launches, and an open panel can stand between the "
                        + "two: " + result.nodes());
    }

    /**
     * The scan behind the planner-follower invariant: every jump edge the
     * search emits has to survive the swept test the launch runs, over the
     * whole configured edge space and with an obstacle at every placement
     * that could interfere.
     */
    @ParameterizedTest
    @CsvSource({"false, false", "false, true", "true, false", "true, true"})
    void everyPlannedJumpSurvivesTheLaunchTheFollowerRuns(
            boolean openableObstacles, boolean opens) {
        MobTraversalProfile profile =
                opens ? OPENER : MobTraversalProfile.DEFAULT;
        Block obstacle = openableObstacles ? CLOSED_GATE : Block.STONE;
        int refused = 0;
        int edges = 0;
        for (int span = 2; span <= 6; span++) {
            for (int rise = -2; rise <= 2; rise++) {
                for (double apex : new double[]{0.5, 1.5, 3.0}) {
                    for (int placement = 0; placement < 127; placement++) {
                        List<PathNode> nodes = scanned(span, rise, apex,
                                placement, obstacle, profile);
                        for (int index = 1; index < nodes.size(); index++) {
                            PathNode landing = nodes.get(index);
                            if (landing.movement()
                                    != PathNode.Movement.JUMP) continue;
                            edges++;
                            if (!launches(span, rise, placement, obstacle,
                                    profile, nodes.get(index - 1), landing,
                                    apex)) refused++;
                        }
                    }
                }
            }
        }
        assertEquals(0, refused, "planner-emitted edges the follower refuses");
        assertTrue(edges > 1_000, "the scan has to reach jump edges at all: "
                + edges);
    }

    @Test
    void aProfileThatOpensNothingKeepsOpenableBlocksOutOfEveryJump() {
        TestWorld world = new TestWorld()
                .set(0, 0, 0, Block.STONE)
                .set(3, 0, 0, Block.STONE)
                .set(3, 1, 0, CLOSED_GATE)
                .set(5, 0, 0, Block.STONE);

        PathResult result = new DiscreteGroundPathfinder().findPath(world, new Pos(0.5, 1, 0.5), new Pos(5.5, 1, 0.5), NARROW, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(48).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD.withPlatformJump(
                        PlatformJumpCapabilities.builder().maxHorizontalDistance(5).maxRise(0).maxDrop(0).apexClearance(1).build()), SearchControl.NONE);

        assertTrue(result.nodes().stream().noneMatch(node ->
                node.graphX() == 3), result.nodes()::toString);
        for (int index = 1; index < result.nodes().size(); index++) {
            PathNode landing = result.nodes().get(index);
            if (landing.movement() != PathNode.Movement.JUMP) continue;
            PathNode takeoff = result.nodes().get(index - 1);
            assertTrue(JumpArc.clear(world, NARROW, takeoff.asVec(),
                            landing.asVec(), 1, null),
                    () -> "a profile with no capability must fly the arc "
                            + "against the world untouched: " + landing);
        }
    }

    private List<PathNode> scanned(int span, int rise, double apex,
                                   int placement, Block obstacle,
                                   MobTraversalProfile profile) {
        return new DiscreteGroundPathfinder().findPath(fixture(span, rise, placement, obstacle, profile), new Pos(0.5, 1, 0.5), new Pos(span + 0.5, rise + 1, 0.5), NARROW, profile, GroundSearchLimits.builder().maxPathLength(48).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD.withPlatformJump(
                                PlatformJumpCapabilities.builder().maxHorizontalDistance(span).maxRise(Math.max(0, rise)).maxDrop(Math.max(0, -rise)).apexClearance(apex).build()), SearchControl.NONE)
                .nodes();
    }

    /** Exactly what the follower does: open the landing, then sweep. */
    private boolean launches(int span, int rise, int placement, Block obstacle,
                             MobTraversalProfile profile, PathNode takeoff,
                             PathNode landing, double apex) {
        TestWorld live = fixture(span, rise, placement, obstacle, profile);
        openLandingColumn(live, profile, landing.asVec());
        return JumpArc.clear(live, NARROW, takeoff.asVec(), landing.asVec(),
                apex, null);
    }

    private static void openLandingColumn(TestWorld world,
                                          MobTraversalProfile profile,
                                          Point waypoint) {
        BlockManipulationCapabilities manipulation =
                profile.blockManipulation();
        if (!manipulation.enabled() || !profile.canPassDoors()) return;
        int x = waypoint.blockX();
        int z = waypoint.blockZ();
        int maxY = waypoint.blockY() + (int) Math.ceil(NARROW.height());
        for (int y = waypoint.blockY(); y <= maxY; y++) {
            Block block = world.getBlock(
                    x, y, z, Block.Getter.Condition.TYPE);
            if (BlockTraversalData.isClosedManipulable(block, manipulation)) {
                world.set(x, y, z, block.withProperty("open", "true"));
            }
        }
    }

    private TestWorld fixture(int span, int rise, int placement,
                              Block obstacle, MobTraversalProfile profile) {
        TestWorld world = new TestWorld()
                .set(0, 0, 0, Block.STONE)
                .set(span, rise, 0, Block.STONE);
        if (profile.blockManipulation().enabled()) {
            world.set(span, rise + 1, 0, CLOSED_GATE);
        }
        // The last placement is the control fixture with nothing in the way.
        if (placement < 126) {
            world.set(1 + placement / 18, -2 + placement % 18, 0, obstacle);
        }
        return world;
    }

    private static TestWorld landing(Block block) {
        TestWorld world = new TestWorld()
                .set(0, 0, 0, Block.STONE)
                .set(3, 0, 0, Block.STONE)
                .set(3, 1, 0, block);
        if (BlockTraversalData.isDoor(block)) {
            world.set(3, 2, 0, block.withProperty("half", "upper"));
        }
        return world;
    }

    private static void assertLanded(PathResult result) {
        assertTrue(result.found(), result::toString);
        assertEquals(PathNode.Movement.JUMP,
                result.nodes().getLast().movement(), result.nodes()::toString);
        assertEquals(3, result.nodes().getLast().graphX());
    }

    private PathResult jump(TestWorld world, MobTraversalProfile profile) {
        return new DiscreteGroundPathfinder().findPath(world, new Pos(0.5, 1, 0.5), new Pos(3.5, 1, 0.5), NARROW, profile, GroundSearchLimits.builder().maxPathLength(48).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD.withPlatformJump(
                        PlatformJumpCapabilities.builder().maxHorizontalDistance(3).maxRise(0).maxDrop(0).apexClearance(1).build()), SearchControl.NONE);
    }
}
