package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Which part of the entity meets a cell decides whether the cell is passable.
 *
 * <p>{@code isLandPathfindable} answers whether feet may occupy a cell, and
 * baseline asks it of every cell the box spans, head included. A carpet is a
 * floor underfoot and a wall at head height, so the cells above the foot layer
 * are measured against the box instead.</p>
 *
 * <p>Every fixture sits in a sealed one-lane corridor at x={@value #LANE},
 * so a route that reaches the far side went through the fixture's cell.</p>
 */
class HeadClearanceTest {
    private static final int LANE = 3;
    private static final int SURFACE = 1;
    private static final double[] WIDTHS = {0.4, 0.6, 0.9, 1.4};
    private static final String[] FACINGS = {"north", "south", "east", "west"};
    private final EntityPathfinder pathfinding = new EntityPathfinder();

    /**
     * A shape that spans its whole cell stands in front of every box, so the
     * width it is met with cannot rescue it.
     */
    @Test
    void aShapeAcrossTheWholeHeadCellIsRefusedAtEveryWidth() {
        List<Block> heads = new ArrayList<>(List.of(
                Block.WHITE_CARPET, Block.LILY_PAD, Block.SCAFFOLDING,
                Block.SNOW.withProperty("layers", "2"),
                Block.SNOW.withProperty("layers", "3"),
                Block.SNOW.withProperty("layers", "4")));
        for (String facing : FACINGS) {
            for (String half : new String[]{"bottom", "top"}) {
                heads.add(trapdoor(facing, half, "false"));
            }
        }
        assertEquals(14, heads.size());

        for (Block head : heads) {
            for (double width : WIDTHS) {
                assertFalse(crossesUnder(head, width),
                        () -> head.key().value() + head.properties()
                                + " at width " + width
                                + " was routed through at head height");
            }
        }
    }

    /** Snow's first layer carries no collision shape at all. */
    @Test
    void anEmptyCollisionShapeOverheadStaysPassable() {
        Block head = Block.SNOW.withProperty("layers", "1");
        for (double width : WIDTHS) {
            assertTrue(crossesUnder(head, width),
                    () -> "an empty shape refused a " + width + " box");
        }
    }

    /**
     * A panel takes one face of its cell. One beside the crossed axis leaves
     * the corridor open until the box is wide enough to reach it: the panel
     * is three sixteenths, so a centred box clears it up to 0.625 wide.
     */
    @Test
    void aPanelBesideTheCrossedAxisIsRefusedOnlyOnceTheBoxReachesIt() {
        List<Block> heads = new ArrayList<>();
        for (String facing : new String[]{"north", "south"}) {
            heads.add(Block.LADDER.withProperty("facing", facing));
            for (String half : new String[]{"bottom", "top"}) {
                heads.add(trapdoor(facing, half, "true"));
            }
        }
        assertEquals(6, heads.size());

        for (Block head : heads) {
            assertTrue(crossesUnder(head, 0.4), head::toString);
            assertTrue(crossesUnder(head, 0.6), head::toString);
            assertTrue(crossesUnder(head, 0.625),
                    () -> "a box that just touches the panel still fits: "
                            + head);
            assertFalse(crossesUnder(head, 0.626), head::toString);
            assertFalse(crossesUnder(head, 0.9), head::toString);
            assertFalse(crossesUnder(head, 1.4), head::toString);
        }
    }

    /**
     * The same panel across the crossed axis walls the corridor off however
     * narrow the entity is, because crossing the cell sweeps its whole width.
     */
    @Test
    void aPanelAcrossTheCrossedAxisIsRefusedAtEveryWidth() {
        List<Block> heads = new ArrayList<>();
        for (String facing : new String[]{"east", "west"}) {
            heads.add(Block.LADDER.withProperty("facing", facing));
            for (String half : new String[]{"bottom", "top"}) {
                heads.add(trapdoor(facing, half, "true"));
            }
        }
        assertEquals(6, heads.size());

        for (Block head : heads) {
            for (double width : WIDTHS) {
                assertFalse(crossesUnder(head, width),
                        () -> head + " at width " + width
                                + " was routed straight into its panel");
            }
        }
    }

    /**
     * A doorway is only ever crossed along the axis its wall is not on, and
     * an open door's panel takes the face the wall already occupies. Judging
     * that panel on the walled axis would seal every doorway in the game.
     */
    @Test
    void anOpenDoorNeverWallsOffTheDoorwayItStandsIn() {
        for (String hinge : new String[]{"left", "right"}) {
            TestWorld world = wallWithDoorway();
            Block door = Block.OAK_DOOR.withProperty("facing", "north")
                    .withProperty("hinge", hinge).withProperty("open", "true");
            world.set(LANE, SURFACE, 0, door.withProperty("half", "lower"));
            world.set(LANE, SURFACE + 1, 0, door.withProperty("half", "upper"));

            PathResult result = pathfinding.findPath(NavigationRequest.builder(
                            world, new Pos(LANE + 0.5, SURFACE, -2.5),
                            new Pos(LANE + 0.5, SURFACE, 2.5),
                            new BoundingBox(0.6, 1.95, 0.6), navigation())
                    .maxPathLength(32).maxVisitedMultiplier(8).build(),
                    SearchControl.NONE);

            assertTrue(result.found(), () -> hinge + ": " + result);
        }
    }

    /** Nothing an entity is short enough to walk under is in its way. */
    @Test
    void aShapeAboveTheBoxIsNotInTheBoxsWay() {
        Block head = trapdoor("north", "top", "false");

        assertTrue(crosses(head, new BoundingBox(0.6, 1.8, 0.6), 1),
                "the panel starts 0.8125 above a head 1.8 up");
        assertFalse(crosses(head, new BoundingBox(0.6, 1.95, 0.6), 1),
                "a 1.95 head reaches into the same panel");
    }

    /**
     * The foot layer keeps the baseline rule verbatim: a shape the entity
     * stands on top of is not a shape standing in it.
     */
    @Test
    void aSurfaceInTheFootCellIsStillWalkedOn() {
        List<Block> surfaces = List.of(
                Block.WHITE_CARPET, Block.LILY_PAD,
                Block.SNOW.withProperty("layers", "4"),
                Block.STONE_SLAB.withProperty("type", "bottom"),
                trapdoor("north", "bottom", "false"));

        for (Block surface : surfaces) {
            for (double width : WIDTHS) {
                assertTrue(crosses(surface,
                        new BoundingBox(width, 1.95, width), 0),
                        () -> surface + " at width " + width
                                + " stopped being walkable underfoot");
            }
        }
    }

    private boolean crossesUnder(Block head, double width) {
        return crosses(head, new BoundingBox(width, 1.95, width), 1);
    }

    private boolean crosses(Block fixture, BoundingBox box, int level) {
        int footprint = Math.max(1, (int) Math.floor(box.width() + 1));
        // A footprint wider than one cell needs a cell of slack to turn in.
        int lane = footprint == 1 ? 1 : footprint + 1;
        TestWorld world = corridor(lane);
        for (int z = 0; z < lane; z++) {
            world.set(LANE, SURFACE + level, z, fixture);
        }
        double anchor = lane * 0.5;
        PathResult result = pathfinding.findPath(NavigationRequest.builder(
                        world, new Pos(anchor, SURFACE, anchor),
                        new Pos(6 + anchor, SURFACE, anchor), box,
                        navigation())
                .maxPathLength(32).maxVisitedMultiplier(8).build(),
                SearchControl.NONE);
        return result.found();
    }

    private static NavigationProfile navigation() {
        return NavigationProfile.builder(NavigationMode.GROUND, MobTraversalProfile.DEFAULT, GroundCapabilities.STANDARD).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();
    }

    private static Block trapdoor(String facing, String half, String open) {
        return Block.OAK_TRAPDOOR.withProperty("facing", facing)
                .withProperty("half", half).withProperty("open", open);
    }

    /** Sealed lane: a refused cell has no detour around it. */
    private static TestWorld corridor(int lane) {
        TestWorld world = new TestWorld().floor(
                -3, 10, -1, lane, SURFACE - 1, Block.STONE);
        for (int x = -3; x <= 10; x++) {
            world.column(x, -1, SURFACE, SURFACE + 4, Block.STONE);
            world.column(x, lane, SURFACE, SURFACE + 4, Block.STONE);
        }
        for (int z = 0; z < lane; z++) {
            world.column(-2, z, SURFACE, SURFACE + 4, Block.STONE);
            world.column(9, z, SURFACE, SURFACE + 4, Block.STONE);
        }
        return world;
    }

    /** A wall along x with one cell missing, crossed along z. */
    private static TestWorld wallWithDoorway() {
        TestWorld world = new TestWorld().floor(
                -1, 8, -4, 4, SURFACE - 1, Block.STONE);
        for (int x = -1; x <= 8; x++) {
            if (x == LANE) continue;
            world.column(x, 0, SURFACE, SURFACE + 4, Block.STONE);
        }
        return world;
    }
}
