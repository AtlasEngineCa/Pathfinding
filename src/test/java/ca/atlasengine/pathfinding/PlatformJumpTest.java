package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.internal.JumpArc;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Boundary contract for the opt-in platform-jump extension. Fixtures are
 * bare islands, so a route exists only when a jump edge is emitted.
 */
class PlatformJumpTest {
    private static final BoundingBox NARROW = new BoundingBox(0.6, 1.8, 0.6);
    /** floor(1.8 + 1) = 2, so the footprint spans two cells per axis. */
    private static final BoundingBox WIDE = new BoundingBox(1.8, 1.8, 1.8);
    private final DiscreteGroundPathfinder pathfinder =
            new DiscreteGroundPathfinder();

    @ParameterizedTest
    @ValueSource(ints = {2, 3, 4, 5, 6})
    void configuredDistanceCrossesExactlyThatManyCellsAndNoMore(
            int maximumDistance) {
        PlatformJumpCapabilities jump = PlatformJumpCapabilities.builder().maxHorizontalDistance(maximumDistance).maxRise(0).maxDrop(0).apexClearance(1).build();

        PathResult exact = jumpPath(islands(maximumDistance),
                new Pos(0.5, 1, 0.5),
                new Pos(maximumDistance + 0.5, 1, 0.5), NARROW, jump);
        PathResult beyond = jumpPath(islands(maximumDistance + 1),
                new Pos(0.5, 1, 0.5),
                new Pos(maximumDistance + 1.5, 1, 0.5), NARROW, jump);

        assertTrue(exact.found(), exact::toString);
        assertEquals(2, exact.nodes().size(),
                "a jump is one atomic supported-to-supported edge");
        assertEquals(PathNode.Movement.JUMP,
                exact.nodes().getLast().movement());
        assertEquals(maximumDistance, exact.nodes().getLast().graphX());
        assertFalse(beyond.found(), beyond::toString);
        assertTrue(beyond.nodes().stream().noneMatch(node ->
                node.movement() == PathNode.Movement.JUMP), beyond::toString);
    }

    @ParameterizedTest
    @ValueSource(doubles = {0, 1, 1.5, 1.999})
    void distancesBelowTwoAreDisabledAndEmitNoJumpEdges(double distance) {
        PlatformJumpCapabilities jump =
                PlatformJumpCapabilities.builder().maxHorizontalDistance(distance).maxRise(4).maxDrop(4).apexClearance(1).build();

        PathResult result = jumpPath(islands(2), new Pos(0.5, 1, 0.5),
                new Pos(2.5, 1, 0.5), NARROW, jump);

        assertFalse(jump.enabled());
        assertFalse(result.found(), result::toString);
        assertTrue(result.nodes().stream().noneMatch(node ->
                node.movement() == PathNode.Movement.JUMP), result::toString);
    }

    @Test
    void disabledConstantIsTheDefaultAndEmitsNoJumpEdges() {
        PathResult result = jumpPath(islands(2), new Pos(0.5, 1, 0.5),
                new Pos(2.5, 1, 0.5), NARROW,
                PlatformJumpCapabilities.DISABLED);

        assertFalse(PlatformJumpCapabilities.DISABLED.enabled());
        assertEquals(PlatformJumpCapabilities.DISABLED,
                GroundCapabilities.STANDARD.platformJump());
        assertFalse(result.found(), result::toString);
        assertTrue(result.nodes().stream().noneMatch(node ->
                node.movement() == PathNode.Movement.JUMP), result::toString);
    }

    @ParameterizedTest
    @ValueSource(ints = {0, 1, 2, 3})
    void configuredRiseIsAnInclusiveBoundaryAtEveryLimit(int maximumRise) {
        PlatformJumpCapabilities jump =
                PlatformJumpCapabilities.builder().maxHorizontalDistance(2).maxRise(maximumRise).maxDrop(0).apexClearance(2).build();

        PathResult exact = jumpPath(islandsAtY(2, maximumRise),
                new Pos(0.5, 1, 0.5), new Pos(2.5, maximumRise + 1, 0.5),
                NARROW, jump);
        PathResult beyond = jumpPath(islandsAtY(2, maximumRise + 1),
                new Pos(0.5, 1, 0.5), new Pos(2.5, maximumRise + 2, 0.5),
                NARROW, jump);

        assertTrue(exact.found(), exact::toString);
        assertEquals(maximumRise + 1, exact.nodes().getLast().graphY());
        assertEquals(PathNode.Movement.JUMP,
                exact.nodes().getLast().movement());
        assertFalse(beyond.found(), beyond::toString);
    }

    @ParameterizedTest
    @ValueSource(ints = {0, 1, 2, 3})
    void configuredDropIsAnInclusiveBoundaryAtEveryLimit(int maximumDrop) {
        PlatformJumpCapabilities jump =
                PlatformJumpCapabilities.builder().maxHorizontalDistance(2).maxRise(0).maxDrop(maximumDrop).apexClearance(2).build();

        PathResult exact = jumpPath(islandsAtY(2, -maximumDrop),
                new Pos(0.5, 1, 0.5), new Pos(2.5, 1 - maximumDrop, 0.5),
                NARROW, jump);
        PathResult beyond = jumpPath(islandsAtY(2, -maximumDrop - 1),
                new Pos(0.5, 1, 0.5), new Pos(2.5, -maximumDrop, 0.5),
                NARROW, jump);

        assertTrue(exact.found(), exact::toString);
        assertEquals(1 - maximumDrop, exact.nodes().getLast().graphY());
        assertEquals(PathNode.Movement.JUMP,
                exact.nodes().getLast().movement());
        assertFalse(beyond.found(), beyond::toString);
    }

    @ParameterizedTest
    @CsvSource({"2, 1", "4, 2"})
    void clearanceIsCheckedAlongTheArcRatherThanTheChord(
            int landingX, int obstacleX) {
        TestWorld world = islands(landingX).set(obstacleX, 3, 0, Block.STONE);
        Pos start = new Pos(0.5, 1, 0.5);
        Pos goal = new Pos(landingX + 0.5, 1, 0.5);

        PathResult chord = jumpPath(world, start, goal, NARROW,
                PlatformJumpCapabilities.builder().maxHorizontalDistance(landingX).maxRise(0).maxDrop(0).apexClearance(0).build());
        PathResult arc = jumpPath(world, start, goal, NARROW,
                PlatformJumpCapabilities.builder().maxHorizontalDistance(landingX).maxRise(0).maxDrop(0).apexClearance(1).build());

        assertTrue(chord.found(),
                () -> "the straight chord is clear of the obstacle: " + chord);
        assertFalse(arc.found(),
                "the raised arc must be sampled with the full bounding box");
    }

    /**
     * The launch climbs the whole rise plus the clearance; the chord the
     * planner used to sweep bowed only {@code apexClearance} off a rising
     * line, so it topped out roughly {@code rise/2} lower. A ceiling in
     * that band is one the mob flies into and nothing ever looked at.
     *
     * <p>The slabs sit above the old chord's own peak, so the control run
     * proves the fixture is jumpable and the obstructed run proves the
     * band is now swept.</p>
     */
    @ParameterizedTest
    @CsvSource({
            "1, 1, 4, 1",
            "2, 1, 5, 1",
            "2, 2, 6, 1",
            "3, 2, 7, 1",
            "4, 2, 8, 1"
    })
    void aCeilingBetweenTheChordAndTheLaunchedApexIsRefused(
            int rise, double apex, int ceilingY, int ceilingX) {
        Block slab = Block.SMOOTH_STONE_SLAB.withProperty("type", "top");
        TestWorld open = islandsAtY(2, rise);
        TestWorld roofed = islandsAtY(2, rise).set(ceilingX, ceilingY, 0, slab);
        PlatformJumpCapabilities jump =
                PlatformJumpCapabilities.builder().maxHorizontalDistance(2).maxRise(rise).maxDrop(0).apexClearance(apex).build();
        Pos start = new Pos(0.5, 1, 0.5);
        Pos goal = new Pos(2.5, rise + 1, 0.5);

        PathResult control = jumpPath(open, start, goal, NARROW, jump);
        PathResult obstructed = jumpPath(roofed, start, goal, NARROW, jump);

        // The slab clears the chord: its underside is above the chord's own
        // peak plus the mob, so the old sweep never reached it.
        assertTrue(ceilingY + 0.5 > 1 + 1.8
                        + (rise + 4 * apex) * (rise + 4 * apex) / (16 * apex),
                () -> "the slab is not above the chord this test replaced");
        assertEquals(PathNode.Movement.JUMP,
                control.nodes().getLast().movement(),
                () -> "the unroofed fixture must stay jumpable: " + control);
        assertFalse(JumpArc.clear(roofed, NARROW, start, goal, apex, null),
                "the follower must refuse the arc under the slab");
        assertFalse(obstructed.found(),
                () -> "the mob flies to " + (1 + rise + apex)
                        + " and the slab starts at " + (ceilingY + 0.5)
                        + ", so the edge is unflyable: "
                        + obstructed.nodes());
    }

    @Test
    void steepArcsAreSampledDenselyEnoughToSeeAnObstacle() {
        PlatformJumpCapabilities steep =
                PlatformJumpCapabilities.builder().maxHorizontalDistance(2).maxRise(0).maxDrop(0).apexClearance(8).build();

        assertTrue(jumpPath(islands(2), new Pos(0.5, 1, 0.5),
                new Pos(2.5, 1, 0.5), NARROW, steep).found());
        assertFalse(jumpPath(islands(2).set(1, 3, 0, Block.STONE),
                        new Pos(0.5, 1, 0.5), new Pos(2.5, 1, 0.5),
                        NARROW, steep).found(),
                "a tall apex must not step the bounding box past a block "
                        + "between two arc samples");
    }

    @Test
    void wideFootprintJumpsBetweenFullySupportedAnchors() {
        TestWorld world = wideIslands(3, 3, 3);
        PlatformJumpCapabilities jump =
                PlatformJumpCapabilities.builder().maxHorizontalDistance(5).maxRise(0).maxDrop(0).apexClearance(1).build();

        PathResult jumped = jumpPath(world, new Pos(1.0, 1, 1.0),
                new Pos(7.0, 1, 1.0), WIDE, jump);
        PathResult tooShort = jumpPath(world, new Pos(1.0, 1, 1.0),
                new Pos(7.0, 1, 1.0), WIDE,
                PlatformJumpCapabilities.builder().maxHorizontalDistance(4).maxRise(0).maxDrop(0).apexClearance(1).build());

        PathNode landing = jumped.nodes().stream().filter(node ->
                        node.movement() == PathNode.Movement.JUMP)
                .findFirst().orElseThrow(() -> new AssertionError(
                        "a two-cell footprint never produced a jump edge: "
                                + jumped));
        assertTrue(jumped.found(), jumped::toString);
        // The last fully supported anchor is x=1; x=2 already overhangs.
        assertEquals(1, jumped.nodes().getFirst().graphX());
        assertEquals(6, landing.graphX(),
                "distance is measured between graph anchors, not gap cells");
        assertEquals(PathStatus.PARTIAL, tooShort.status());
        assertTrue(tooShort.nodes().stream().noneMatch(node ->
                        node.movement() == PathNode.Movement.JUMP),
                tooShort::toString);
    }

    @Test
    void wideFootprintRejectsALandingNarrowerThanItself() {
        TestWorld world = wideIslands(3, 3, 3);
        for (int x = 6; x < 9; x++) world.set(x, 0, 0, Block.AIR);
        for (int x = 6; x < 9; x++) world.set(x, 0, 2, Block.AIR);
        PlatformJumpCapabilities jump =
                PlatformJumpCapabilities.builder().maxHorizontalDistance(6).maxRise(0).maxDrop(0).apexClearance(1).build();

        PathResult result = jumpPath(world, new Pos(1.0, 1, 1.0),
                new Pos(7.0, 1, 1.0), WIDE, jump);

        assertEquals(PathStatus.PARTIAL, result.status());
        assertTrue(result.nodes().stream().noneMatch(node ->
                        node.movement() == PathNode.Movement.JUMP),
                () -> "a one-cell-deep strip cannot support a 2x2 footprint: "
                        + result.nodes());
    }

    @Test
    void wideFootprintTakeoffMustBeFullySupported() {
        // Only the far half of the takeoff island is present, so every anchor
        // that could reach the landing overhangs the void.
        TestWorld world = new TestWorld();
        for (int z = 0; z < 3; z++) {
            world.set(0, 0, z, Block.STONE);
            for (int x = 6; x < 9; x++) world.set(x, 0, z, Block.STONE);
        }
        PlatformJumpCapabilities jump =
                PlatformJumpCapabilities.builder().maxHorizontalDistance(8).maxRise(0).maxDrop(0).apexClearance(1).build();

        PathResult result = jumpPath(world, new Pos(0.5, 1, 1.0),
                new Pos(7.0, 1, 1.0), WIDE, jump);

        assertTrue(result.nodes().stream().noneMatch(node ->
                        node.movement() == PathNode.Movement.JUMP),
                () -> "a one-cell island cannot support a 2x2 takeoff: "
                        + result.nodes());
    }

    @ParameterizedTest
    @CsvSource({
            "stone, true",
            "smooth_stone_slab[type=top], true",
            "smooth_stone_slab[type=bottom], false",
            "farmland, false"
    })
    void onlyFullBlockSupportCanLaunchOrReceiveAJump(
            String support, boolean jumpable) {
        Block block = Block.fromState(support);
        assertNotNull(block, support);
        PlatformJumpCapabilities jump =
                PlatformJumpCapabilities.builder().maxHorizontalDistance(2).maxRise(1).maxDrop(1).apexClearance(1).build();
        TestWorld takeoff = new TestWorld()
                .set(0, 0, 0, block).set(2, 0, 0, Block.STONE);
        TestWorld landing = new TestWorld()
                .set(0, 0, 0, Block.STONE).set(2, 0, 0, block);

        assertEquals(jumpable, jumpPath(takeoff, new Pos(0.5, 1, 0.5),
                new Pos(2.5, 1, 0.5), NARROW, jump).found(), support);
        assertEquals(jumpable, jumpPath(landing, new Pos(0.5, 1, 0.5),
                new Pos(2.5, 1, 0.5), NARROW, jump).found(), support);
    }

    @Test
    void continuousGroundNeverProducesJumpEdges() {
        TestWorld world = new TestWorld()
                .floor(-2, 10, -2, 2, 0, Block.STONE)
                .set(3, 1, 0, Block.WHITE_CARPET)
                .set(4, 1, 0, Block.STONE_SLAB.withProperty("type", "bottom"))
                .set(5, 1, 0, Block.STONE_STAIRS
                        .withProperty("facing", "east")
                        .withProperty("half", "bottom"));

        PathResult result = jumpPath(world, new Pos(0.5, 1, 0.5),
                new Pos(8.5, 1, 0.5), NARROW,
                PlatformJumpCapabilities.builder().maxHorizontalDistance(6).maxRise(2).maxDrop(2).apexClearance(1).build());

        PathResult wide = jumpPath(
                new TestWorld().floor(-2, 10, -2, 6, 0, Block.STONE),
                new Pos(1.0, 1, 1.0), new Pos(8.0, 1, 1.0), WIDE,
                PlatformJumpCapabilities.builder().maxHorizontalDistance(6).maxRise(2).maxDrop(2).apexClearance(1).build());

        assertTrue(result.found(), result::toString);
        assertTrue(result.nodes().stream().noneMatch(node ->
                        node.movement() == PathNode.Movement.JUMP),
                () -> "ordinary terrain became jump edges: " + result.nodes());
        for (int index = 1; index < result.nodes().size(); index++) {
            PathNode previous = result.nodes().get(index - 1);
            PathNode node = result.nodes().get(index);
            assertTrue(Math.abs(node.graphX() - previous.graphX()) <= 1
                            && Math.abs(node.graphZ() - previous.graphZ()) <= 1,
                    result.nodes()::toString);
        }
        assertTrue(wide.found(), wide::toString);
        assertTrue(wide.nodes().stream().noneMatch(node ->
                        node.movement() == PathNode.Movement.JUMP),
                () -> "a wide footprint jumped across solid ground: "
                        + wide.nodes());
    }

    /**
     * The follower sweeps the arc before it launches, and a refusal leaves
     * the entity standing on the takeoff, so the recomputation it triggers
     * re-emits the same edge forever. Point-sampling the arc lets the box
     * step diagonally past a block corner between two samples, which is
     * exactly what these obstacles sit on.
     */
    @ParameterizedTest
    @CsvSource({
            "5, 1, 2, 0, 4",
            "5, 1, 2, 1, 5",
            "5, 0, 6, 0, 6",
            "5, -1, 2, 5, 3",
            "5, -1, 2, 4, 4",
            // The rising arc's shoulder, one cell higher than the chord's.
            "4, 2, 4, 2, 5",
            "6, 1, 3, 6, 5"
    })
    void everyEmittedJumpSurvivesTheSweptTestTheFollowerRuns(
            int landingX, int rise, double apex,
            int obstacleX, int obstacleY) {
        TestWorld world = islandsAtY(landingX, rise)
                .set(obstacleX, obstacleY, 0, Block.STONE);
        PlatformJumpCapabilities jump = PlatformJumpCapabilities.builder().maxHorizontalDistance(landingX).maxRise(Math.max(0, rise)).maxDrop(Math.max(0, -rise)).apexClearance(apex).build();
        Pos start = new Pos(0.5, 1, 0.5);
        Pos goal = new Pos(landingX + 0.5, rise + 1, 0.5);

        PathResult control = jumpPath(
                islandsAtY(landingX, rise), start, goal, NARROW, jump);
        PathResult obstructed = jumpPath(world, start, goal, NARROW, jump);

        assertEquals(PathNode.Movement.JUMP,
                control.nodes().getLast().movement(),
                () -> "the obstacle-free fixture must be jumpable: " + control);
        assertFalse(JumpArc.clear(world, NARROW, start, goal, apex, null),
                "the obstacle must be one the follower refuses to fly past");
        assertFalse(obstructed.found(),
                () -> "the planner emitted an edge the follower refuses, so "
                        + "the entity would stand still forever: "
                        + obstructed.nodes());
        for (int index = 1; index < obstructed.nodes().size(); index++) {
            PathNode landing = obstructed.nodes().get(index);
            if (landing.movement() != PathNode.Movement.JUMP) continue;
            PathNode takeoff = obstructed.nodes().get(index - 1);
            assertTrue(JumpArc.clear(world, NARROW, takeoff.asVec(),
                            landing.asVec(), apex, null),
                    () -> "unflyable edge " + takeoff.asVec() + " -> "
                            + landing.asVec());
        }
    }

    /**
     * Supported and walkable are different things. An island the mob may
     * not stand on is flown over like any other gap cell, and the arc is
     * still swept, so the clearance is the block's own collision shape.
     */
    @ParameterizedTest
    @CsvSource({"cactus, 2", "oak_fence, 2", "stone, 3"})
    void aJumpClearsASupportedIslandTheMobMayNotStandOn(
            String island, double apex) {
        Block block = Block.fromState(island);
        assertNotNull(block, island);
        TestWorld world = islands(4).set(2, 0, 0, Block.STONE)
                .set(2, 1, 0, block);
        if (island.equals("stone")) world.set(2, 2, 0, Block.STONE);
        PlatformJumpCapabilities jump =
                PlatformJumpCapabilities.builder().maxHorizontalDistance(4).maxRise(0).maxDrop(0).apexClearance(apex).build();

        PathResult result = jumpPath(world, new Pos(0.5, 1, 0.5),
                new Pos(4.5, 1, 0.5), NARROW, jump);

        assertTrue(result.found(), result::toString);
        assertEquals(2, result.nodes().size(), result.nodes()::toString);
        assertEquals(PathNode.Movement.JUMP,
                result.nodes().getLast().movement());
        assertEquals(4, result.nodes().getLast().graphX());
        assertTrue(JumpArc.clear(world, NARROW,
                        result.nodes().getFirst().asVec(),
                        result.nodes().getLast().asVec(), apex, null),
                "the arc over the island must survive the swept test");
        assertFalse(jumpPath(world, new Pos(0.5, 1, 0.5),
                        new Pos(4.5, 1, 0.5), NARROW,
                        PlatformJumpCapabilities.builder().maxHorizontalDistance(4).maxRise(0).maxDrop(0).apexClearance(0.25).build()).found(),
                "a flat arc must still be refused by the swept test");
    }

    @Test
    void aSupportedWalkableIslandIsLandedOnRatherThanFlownOver() {
        TestWorld world = islands(4).set(2, 0, 0, Block.STONE);
        PlatformJumpCapabilities jump =
                PlatformJumpCapabilities.builder().maxHorizontalDistance(4).maxRise(0).maxDrop(0).apexClearance(1).build();

        PathResult result = jumpPath(world, new Pos(0.5, 1, 0.5),
                new Pos(4.5, 1, 0.5), NARROW, jump);

        assertTrue(result.found(), result::toString);
        assertEquals(List.of(0, 2, 4), result.nodes().stream()
                        .map(PathNode::graphX).toList(),
                () -> "walkable ground must stay a landing: "
                        + result.nodes());
    }

    /**
     * The island's cost is the mob's own. A profile that cannot enter the
     * block flies over it; one that can keeps the cell as ordinary terrain.
     */
    @Test
    void anIslandImpassableForThisProfileIsJumpedOverAndNeverEnteredNothingElse() {
        TestWorld world = islands(4).set(2, 0, 0, Block.STONE)
                .set(2, 1, 0, Block.WATER);
        PlatformJumpCapabilities jump =
                PlatformJumpCapabilities.builder().maxHorizontalDistance(4).maxRise(0).maxDrop(0).apexClearance(1).build();

        PathResult avoidsWater = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(4.5, 1, 0.5), NARROW, MobTraversalProfile.ENDERMAN, GroundSearchLimits.builder().maxPathLength(48).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD.withPlatformJump(jump), SearchControl.NONE);
        PathResult wadesWater = jumpPath(world, new Pos(0.5, 1, 0.5),
                new Pos(4.5, 1, 0.5), NARROW, jump);

        assertTrue(avoidsWater.found(), avoidsWater::toString);
        assertEquals(PathNode.Movement.JUMP,
                avoidsWater.nodes().getLast().movement());
        assertTrue(avoidsWater.nodes().stream().noneMatch(node ->
                        node.graphX() == 2),
                () -> "routed onto an impassable island: "
                        + avoidsWater.nodes());
        assertTrue(wadesWater.nodes().stream().noneMatch(node ->
                        node.movement() == PathNode.Movement.JUMP),
                () -> "a profile that may enter the island keeps it as "
                        + "ordinary terrain: " + wadesWater.nodes());
    }

    @Test
    void capabilityRecordRejectsOutOfRangeAndNonFiniteValues() {
        assertThrows(IllegalArgumentException.class,
                () -> PlatformJumpCapabilities.builder().maxHorizontalDistance(-1).maxRise(0).maxDrop(0).apexClearance(0).build());
        assertThrows(IllegalArgumentException.class,
                () -> PlatformJumpCapabilities.builder().maxHorizontalDistance(33).maxRise(0).maxDrop(0).apexClearance(0).build());
        assertThrows(IllegalArgumentException.class,
                () -> PlatformJumpCapabilities.builder().maxHorizontalDistance(2).maxRise(17).maxDrop(0).apexClearance(0).build());
        assertThrows(IllegalArgumentException.class,
                () -> PlatformJumpCapabilities.builder().maxHorizontalDistance(2).maxRise(0).maxDrop(0).apexClearance(Double.NaN).build());
        assertThrows(IllegalArgumentException.class,
                () -> PlatformJumpCapabilities.acrossGaps(-1));
    }

    @ParameterizedTest
    @ValueSource(ints = {1, 2, 3, 4})
    void acrossGapsSizesTheLimitByUnsupportedCells(int gapCells) {
        PlatformJumpCapabilities jump =
                PlatformJumpCapabilities.acrossGaps(gapCells);

        PathResult exact = jumpPath(islands(gapCells + 1),
                new Pos(0.5, 1, 0.5), new Pos(gapCells + 1.5, 1, 0.5),
                NARROW, jump);
        PathResult beyond = jumpPath(islands(gapCells + 2),
                new Pos(0.5, 1, 0.5), new Pos(gapCells + 2.5, 1, 0.5),
                NARROW, jump);

        assertTrue(jump.enabled());
        assertEquals(gapCells + 1, jump.maxHorizontalDistance());
        assertTrue(exact.found(), exact::toString);
        assertEquals(PathNode.Movement.JUMP,
                exact.nodes().getLast().movement());
        assertFalse(beyond.found(), beyond::toString);
        assertFalse(PlatformJumpCapabilities.acrossGaps(0).enabled(),
                "a zero-cell gap needs no jump edge");
    }

    @Test
    void withersLayerRiseDropAndClearanceOntoAGapSizedBase() {
        PlatformJumpCapabilities jump = PlatformJumpCapabilities.acrossGaps(2)
                .withRise(1).withDrop(3).withApexClearance(2);

        assertEquals(PlatformJumpCapabilities.builder().maxHorizontalDistance(3).maxRise(1).maxDrop(3).apexClearance(2).build(), jump);
        assertTrue(jumpPath(islandsAtY(3, 1), new Pos(0.5, 1, 0.5),
                new Pos(3.5, 2, 0.5), NARROW, jump).found());
        assertTrue(jumpPath(islandsAtY(3, -3), new Pos(0.5, 1, 0.5),
                new Pos(3.5, -2, 0.5), NARROW, jump).found());
        assertFalse(jumpPath(islandsAtY(3, 2), new Pos(0.5, 1, 0.5),
                new Pos(3.5, 3, 0.5), NARROW, jump).found());
    }

    private PathResult jumpPath(TestWorld world, Pos start, Pos goal,
                                BoundingBox box,
                                PlatformJumpCapabilities jump) {
        return pathfinder.findPath(world, start, goal, box, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(48).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD.withPlatformJump(jump), SearchControl.NONE);
    }

    private static TestWorld islands(int landingX) {
        return new TestWorld()
                .set(0, 0, 0, Block.STONE)
                .set(landingX, 0, 0, Block.STONE);
    }

    private static TestWorld islandsAtY(int landingX, int landingSupportY) {
        return new TestWorld()
                .set(0, 0, 0, Block.STONE)
                .set(landingX, landingSupportY, 0, Block.STONE);
    }

    private static TestWorld wideIslands(int platform, int gap, int depth) {
        TestWorld world = new TestWorld();
        int landing = platform + gap;
        for (int z = 0; z < depth; z++) {
            for (int x = 0; x < platform; x++) world.set(x, 0, z, Block.STONE);
            for (int x = landing; x < landing + platform; x++) {
                world.set(x, 0, z, Block.STONE);
            }
        }
        return world;
    }
}
