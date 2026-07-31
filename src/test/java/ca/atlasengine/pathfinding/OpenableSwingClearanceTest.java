package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.OpenableBlockFamily;
import ca.atlasengine.pathfinding.terrain.TerrainClassifier;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Which way an openable swings, and how wide the entity is.
 *
 * <p>An open trapdoor or door is a three-sixteenths panel across one face of
 * its cell, not air. The face it takes decides which axis of the cell stays
 * crossable, and the thirteen-sixteenths it leaves decides who fits: a
 * centred box clears it only up to 0.625 wide. A fence gate opens to an empty
 * shape and never obstructs anyone.</p>
 */
class OpenableSwingClearanceTest {
    private static final BoundingBox NARROW = new BoundingBox(0.6, 1.95, 0.6);
    private static final BoundingBox MEDIUM = new BoundingBox(0.9, 1.95, 0.9);
    private static final BoundingBox WIDE = new BoundingBox(1.4, 1.95, 1.4);
    private static final String[] FACINGS = {"north", "south", "east", "west"};
    private static final String[] HALVES = {"bottom", "top"};
    private final EntityPathfinder pathfinding = new EntityPathfinder();
    private final TerrainClassifier classifier = new TerrainClassifier();

    /**
     * The corridor runs along +x, so a panel that swings across x walls it
     * off and a panel that swings across z leaves it open. Both trapdoor
     * halves swing to the same face, and a panel is full height, so the
     * answer may not depend on the half or on the level it sits at.
     */
    @Test
    void everyTrapdoorOrientationEitherCrossesOrIsPlannedAround() {
        List<String> outcomes = new ArrayList<>();
        for (String facing : FACINGS) {
            for (String half : HALVES) {
                for (int level : new int[]{1, 2}) {
                    TestWorld world = corridor().set(
                            2, level, 0, trapdoor(facing, half));
                    PathResult result = cross(world, NARROW, trapdoors());
                    int last = result.nodes().getLast().graphX();
                    outcomes.add(facing + "/" + half + "/y" + level + " "
                            + (result.found() ? "crossed" : "stopped at " + last));
                }
            }
        }

        // North and south swing across z and leave the corridor open. East
        // swings onto the cell's own west face, so the cell cannot even be
        // entered from x=1; west swings onto its east face, so the cell is
        // enterable but is a dead end. Neither half nor level changes a panel.
        assertEquals(List.of(
                "north/bottom/y1 crossed", "north/bottom/y2 crossed",
                "north/top/y1 crossed", "north/top/y2 crossed",
                "south/bottom/y1 crossed", "south/bottom/y2 crossed",
                "south/top/y1 crossed", "south/top/y2 crossed",
                "east/bottom/y1 stopped at 1", "east/bottom/y2 stopped at 1",
                "east/top/y1 stopped at 1", "east/top/y2 stopped at 1",
                "west/bottom/y1 stopped at 2", "west/bottom/y2 stopped at 2",
                "west/top/y1 stopped at 2", "west/top/y2 stopped at 2"),
                outcomes);
    }

    /** The one cell, entered across the axis its panel leaves clear. */
    @Test
    void theCellStaysEnterableAcrossTheAxisThePanelLeavesClear() {
        Block panel = trapdoor("east", "bottom");
        PathResult alongPanel = pathfinding.findPath(NavigationRequest.builder(
                        zCorridor().set(2, 1, 0, panel), new Pos(2.5, 1, -2.5),
                        new Pos(2.5, 1, 2.5), NARROW, navigation(trapdoors()))
                .maxPathLength(32).maxVisitedMultiplier(8).build(),
                SearchControl.NONE);
        PathResult acrossPanel = cross(
                corridor().set(2, 1, 0, panel), NARROW, trapdoors());

        assertTrue(alongPanel.found(), alongPanel::toString);
        assertTrue(alongPanel.nodes().stream().anyMatch(
                        node -> node.graphX() == 2 && node.graphZ() == 0),
                alongPanel::toString);
        assertFalse(acrossPanel.found(),
                "the same panel walls off the axis it swung across");
    }

    /**
     * A centred box clears the thirteen-sixteenths a panel leaves exactly up
     * to 0.625 wide. Touching is not intersecting, so the boundary case fits.
     */
    @Test
    void panelClearanceFollowsTheEntityWidthToTheBlockBoundary() {
        TestWorld world = corridor().set(2, 1, 0, trapdoor("north", "bottom"));

        assertTrue(cross(world, NARROW, trapdoors()).found());
        assertTrue(cross(world, box(0.625), trapdoors()).found(),
                "a box that just touches the panel still fits past it");
        assertFalse(cross(world, box(0.626), trapdoors()).found());
        assertFalse(cross(world, MEDIUM, trapdoors()).found());
    }

    /**
     * A panel already standing open is the same panel. The follower has
     * nothing to toggle there, but the search still has to fit past it, so a
     * manipulator reads an open one exactly as it reads one it would open.
     */
    @Test
    void anAlreadyOpenPanelCountsForTheFamiliesTheProfileManipulates() {
        TestWorld world = corridor().set(2, 1, 0, Block.OAK_TRAPDOOR
                .withProperty("facing", "east").withProperty("half", "bottom")
                .withProperty("open", "true"));

        assertFalse(cross(world, NARROW, trapdoors()).found());
        assertTrue(cross(world, NARROW, MobTraversalProfile.DEFAULT).found(),
                "baseline prices any trapdoor at zero, open or shut");
    }

    /** A cell whose panel stands in the resting box is no cell to stand in. */
    @Test
    void aPanelInsideTheRestingBoxBlocksTheCellOutright() {
        TestWorld world = corridor().set(2, 2, 0, trapdoor("north", "bottom"));

        assertEquals(TerrainType.WALKABLE_DOOR,
                classify(world, 2, 1, 0, NARROW, trapdoors()));
        assertEquals(TerrainType.BLOCKED,
                classify(world, 2, 1, 0, MEDIUM, trapdoors()));
        assertEquals(TerrainType.BLOCKED,
                classify(world, 2, 1, 0, MEDIUM, doorsOnly()),
                "a family this profile never opens keeps its physical shape");
        assertEquals(TerrainType.BLOCKED,
                classify(world, 2, 1, 0, MEDIUM, MobTraversalProfile.DEFAULT));
    }

    @Test
    void aTwoCellFootprintIsRefusedByEveryTrapdoorOrientation() {
        for (String facing : FACINGS) {
            TestWorld world = gap(trapdoor(facing, "bottom"));
            PathResult result = crossGap(world, WIDE, trapdoors());

            assertFalse(result.found(), () -> facing + ": " + result);
            assertTrue(result.nodes().stream().allMatch(
                            node -> node.graphX() < 3),
                    () -> facing + " planned into the panel: " + result);
        }
    }

    /**
     * A door swings the same way, so it is judged the same way. Its closed
     * panel lies across the doorway and its open one lies along it, which is
     * why a door hung across a corridor rather than in it is the orientation
     * that walls the corridor off once opened.
     */
    @Test
    void doorsAreJudgedByTheSwungPanelToo() {
        for (String hinge : new String[]{"left", "right"}) {
            assertTrue(cross(corridor(door("east", hinge)), NARROW,
                    doorsOnly()).found(), hinge);
            assertFalse(cross(corridor(door("north", hinge)), NARROW,
                    doorsOnly()).found(), hinge);
            assertFalse(cross(corridor(door("east", hinge)), MEDIUM,
                    doorsOnly()).found(), hinge);
        }
        assertFalse(crossGap(gapDoor(), WIDE, doorsOnly()).found());
    }

    /** An open fence gate has an empty collision shape. */
    @Test
    void fenceGatesOpenToNothingSoEveryOrientationCrosses() {
        for (String facing : FACINGS) {
            TestWorld world = corridor().set(2, 1, 0, gate(facing));

            assertTrue(cross(world, NARROW, gates()).found(), facing);
            assertTrue(cross(world, MEDIUM, gates()).found(), facing);
            assertTrue(crossGap(gap(gate(facing)), WIDE, gates()).found(),
                    facing);
        }
    }

    /**
     * An entity that is already standing in a panel keeps every edge it had.
     * Moving cannot make it worse off, and refusing all four would leave it
     * with nowhere to go at all.
     */
    @Test
    void anEntityAlreadyInsideAPanelCanStillWalkOutOfIt() {
        TestWorld world = corridor().set(2, 1, 0, Block.OAK_TRAPDOOR
                .withProperty("facing", "north").withProperty("half", "bottom")
                .withProperty("open", "true"));

        PathResult result = pathfinding.findPath(NavigationRequest.builder(
                        world, new Pos(2.5, 1, 0.5), new Pos(0.5, 1, 0.5),
                        MEDIUM, navigation(trapdoors()))
                .maxPathLength(32).maxVisitedMultiplier(8).build(),
                SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        assertEquals(0, result.nodes().getLast().graphX());
    }

    /**
     * Nothing is opened for a profile that cannot pass doors, so the closed
     * shape stands and the baseline path type is what the search reads.
     */
    @Test
    void aProfileThatCannotPassDoorsIsNeverJudgedOnASwungPanel() {
        TestWorld world = corridor().set(2, 2, 0, trapdoor("north", "bottom"));
        MobTraversalProfile shut = MobTraversalProfile.builder("shut")
                .blockManipulation(BlockManipulationCapabilities.of(
                        OpenableBlockFamily.values()))
                .canPassDoors(false)
                .build();

        assertEquals(TerrainType.BLOCKED,
                classify(world, 2, 1, 0, MEDIUM, shut));
    }

    private PathResult cross(TestWorld world, BoundingBox box,
                             MobTraversalProfile profile) {
        return pathfinding.findPath(NavigationRequest.builder(world,
                        new Pos(0.5, 1, 0.5), new Pos(4.5, 1, 0.5), box,
                        navigation(profile))
                .maxPathLength(32).maxVisitedMultiplier(8).build(),
                SearchControl.NONE);
    }

    /** Two-cell footprints anchor on the corner, so they start at a corner. */
    private PathResult crossGap(TestWorld world, BoundingBox box,
                                MobTraversalProfile profile) {
        return pathfinding.findPath(NavigationRequest.builder(world,
                        new Pos(1.0, 1, 0.0), new Pos(6.0, 1, 0.0), box,
                        navigation(profile))
                .maxPathLength(32).maxVisitedMultiplier(8).build(),
                SearchControl.NONE);
    }

    private TerrainType classify(TestWorld world, int x, int y, int z,
                                 BoundingBox box, MobTraversalProfile profile) {
        return classifier.classifyAnchored(world, x, y, z, box, profile);
    }

    private static NavigationProfile navigation(MobTraversalProfile profile) {
        return BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE)
                .withMobProfile(profile);
    }

    private static MobTraversalProfile manipulating(
            OpenableBlockFamily... families) {
        return MobTraversalProfile.builder("manipulating")
                .blockManipulation(BlockManipulationCapabilities.of(families))
                .build();
    }

    private static MobTraversalProfile trapdoors() {
        return manipulating(OpenableBlockFamily.TRAPDOOR);
    }

    private static MobTraversalProfile doorsOnly() {
        return manipulating(OpenableBlockFamily.DOOR);
    }

    private static MobTraversalProfile gates() {
        return manipulating(OpenableBlockFamily.FENCE_GATE);
    }

    private static BoundingBox box(double width) {
        return new BoundingBox(width, 1.95, width);
    }

    private static Block trapdoor(String facing, String half) {
        return Block.OAK_TRAPDOOR.withProperty("facing", facing)
                .withProperty("half", half).withProperty("open", "false");
    }

    private static Block door(String facing, String hinge) {
        return Block.OAK_DOOR.withProperty("facing", facing)
                .withProperty("hinge", hinge).withProperty("open", "false");
    }

    private static Block gate(String facing) {
        return Block.OAK_FENCE_GATE.withProperty("facing", facing)
                .withProperty("open", "false");
    }

    /** Sealed one-wide corridor along +x: no configuration has a detour. */
    private static TestWorld corridor() {
        TestWorld world = new TestWorld().floor(-1, 5, -1, 1, 0, Block.STONE);
        for (int x = -1; x <= 5; x++) {
            world.column(x, -1, 1, 4, Block.STONE);
            world.column(x, 1, 1, 4, Block.STONE);
        }
        for (int z = -1; z <= 1; z++) {
            world.column(-1, z, 1, 4, Block.STONE);
            world.column(5, z, 1, 4, Block.STONE);
        }
        return world;
    }

    private static TestWorld corridor(Block door) {
        TestWorld world = corridor();
        world.set(2, 1, 0, door.withProperty("half", "lower"));
        world.set(2, 2, 0, door.withProperty("half", "upper"));
        return world;
    }

    /** Sealed corridor along +z through the same cell. */
    private static TestWorld zCorridor() {
        TestWorld world = new TestWorld().floor(1, 3, -3, 3, 0, Block.STONE);
        for (int z = -3; z <= 3; z++) {
            world.column(1, z, 1, 4, Block.STONE);
            world.column(3, z, 1, 4, Block.STONE);
        }
        for (int x = 1; x <= 3; x++) {
            world.column(x, -3, 1, 4, Block.STONE);
            world.column(x, 3, 1, 4, Block.STONE);
        }
        return world;
    }

    /** Three-wide wall opening, so a two-cell footprint can reach it. */
    private static TestWorld gap(Block openable) {
        TestWorld world = new TestWorld().floor(-1, 7, -3, 3, 0, Block.STONE);
        for (int z = -3; z <= 3; z++) {
            if (z >= -1 && z <= 1) continue;
            world.column(3, z, 1, 4, Block.STONE);
        }
        for (int z = -1; z <= 1; z++) world.set(3, 1, z, openable);
        return world;
    }

    private static TestWorld gapDoor() {
        TestWorld world = gap(Block.AIR);
        Block base = door("east", "left");
        for (int z = -1; z <= 1; z++) {
            world.set(3, 1, z, base.withProperty("half", "lower"));
            world.set(3, 2, z, base.withProperty("half", "upper"));
        }
        return world;
    }
}
