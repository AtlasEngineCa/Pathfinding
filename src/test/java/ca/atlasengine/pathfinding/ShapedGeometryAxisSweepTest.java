package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainClassifier;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Planner sweeps over the shape-bearing block axes. Every fixture is placed
 * in a sealed one-wide corridor, so a route that reaches the far end must
 * have gone through the cell under test and a refusal cannot be a detour.
 *
 * <p>Widen the width sweep with {@code -Dpathfinding.shapedWidthScenarios=all}.
 * The default runs the two widths that bracket a one-cell footprint.</p>
 */
class ShapedGeometryAxisSweepTest {
    private static final String WIDTH_PROPERTY =
            "pathfinding.shapedWidthScenarios";
    private static final double[] DEFAULT_WIDTHS = {0.6, 0.9};
    private static final double[] ALL_WIDTHS = {0.4, 0.6, 0.626, 0.9};
    private static final String[] FACINGS =
            {"north", "south", "east", "west"};
    private static final String[] SHAPES = {"straight", "inner_left",
            "inner_right", "outer_left", "outer_right"};
    private static final BoundingBox STANDARD =
            new BoundingBox(0.6, 1.95, 0.6);

    private final DiscreteGroundPathfinder pathfinder =
            new DiscreteGroundPathfinder();

    private static double[] widths() {
        return "all".equals(System.getProperty(WIDTH_PROPERTY))
                ? ALL_WIDTHS : DEFAULT_WIDTHS;
    }

    @Test
    void everyStairStateBlocksItsCellAndProjectsItsOwnCollisionTop() {
        List<Block> states = stairStates();
        assertEquals(80, states.size(), "stair axis lost states");
        int stepped = 0;
        for (Block stair : states) {
            assertEquals(TerrainType.BLOCKED, TerrainClassifier.raw(stair),
                    () -> "stair became routable terrain: " + stair);
            for (double width : widths()) {
                PathResult result = crossLane(stair, 1, width);
                assertTrue(result.found(),
                        () -> "no route over " + stair + " at " + width);
                PathNode onTop = nodeAt(result, 3);
                assertNotNull(onTop, () -> "route dodged the stair: "
                        + result.nodes());
                assertEquals(2, onTop.graphY(),
                        () -> "stair top is not a graph step: " + onTop);
                assertEquals(2.0, onTop.y(), 1.0e-9,
                        () -> "stair waypoint left its own surface: " + onTop);
                assertEquals(PathNode.Movement.STEP_UP, onTop.movement());
                stepped++;
            }
        }
        assertEquals(states.size() * widths().length, stepped);
    }

    @Test
    void slabAndSnowWaypointsTrackTheirRealCollisionSurface() {
        Map<Block, Double> expected = new LinkedHashMap<>();
        for (String type : new String[]{"top", "bottom", "double"}) {
            Block slab = Block.STONE_SLAB.withProperty("type", type);
            expected.put(slab, surface(slab));
            expected.put(slab.withProperty("waterlogged", "true"),
                    surface(slab));
        }
        for (int layers = 5; layers <= 8; layers++) {
            Block snow = Block.SNOW.withProperty(
                    "layers", String.valueOf(layers));
            expected.put(snow, surface(snow));
        }

        assertEquals(10, expected.size());
        expected.forEach((block, top) -> {
            assertEquals(TerrainType.BLOCKED, TerrainClassifier.raw(block),
                    () -> "expected an obstacle: " + block);
            PathResult result = crossLane(block, 1, 0.6);
            assertTrue(result.found(), () -> "no route over " + block);
            PathNode onTop = nodeAt(result, 3);
            assertNotNull(onTop, () -> "route dodged " + block);
            assertEquals(1 + top, onTop.y(), 1.0e-9,
                    () -> "waypoint ignored the collision surface of "
                            + block + ": " + onTop);
        });
    }

    @Test
    void snowBecomesAnObstacleExactlyAtTheFifthLayer() {
        for (int layers = 1; layers <= 8; layers++) {
            Block snow = Block.SNOW.withProperty(
                    "layers", String.valueOf(layers));
            TerrainType type = TerrainClassifier.raw(snow);
            int expectedLayers = layers;
            assertEquals(expectedLayers < 5
                            ? TerrainType.OPEN : TerrainType.BLOCKED, type,
                    () -> "snow layer rule moved at " + expectedLayers);
            PathResult result = crossLane(snow, 1, 0.6);
            assertTrue(result.found(), () -> "no route over " + snow);
            PathNode at = nodeAt(result, 3);
            assertNotNull(at, () -> "route dodged " + snow);
            // Below five layers the cell is walked, not stepped onto, so the
            // waypoint comes from the floor beneath rather than the snow.
            assertEquals(expectedLayers < 5 ? 1.0 : 1 + surface(snow), at.y(),
                    1.0e-9, () -> "snow waypoint: " + at);
        }
    }

    @Test
    void aFullCubeAtHeadHeightIsRefusedForEveryShapeThatCarriesOne() {
        List<Block> heads = new ArrayList<>(stairStates());
        for (String type : new String[]{"top", "bottom", "double"}) {
            heads.add(Block.STONE_SLAB.withProperty("type", type));
        }
        for (int layers = 5; layers <= 8; layers++) {
            heads.add(Block.SNOW.withProperty(
                    "layers", String.valueOf(layers)));
        }
        heads.add(Block.STONE);

        assertEquals(88, heads.size());
        for (Block head : heads) {
            for (double width : widths()) {
                PathResult result = crossLane(head, 2, width);
                assertNull(nodeAt(result, 3),
                        () -> "planned through a blocked head cell of "
                                + head + " at width " + width + ": "
                                + result.nodes());
            }
        }
    }

    @Test
    void railsStayImpassableUntilTheProfileIsRidingOne() {
        List<Block> rails = new ArrayList<>();
        for (String shape : new String[]{"north_south", "east_west",
                "ascending_east", "ascending_west", "ascending_north",
                "ascending_south", "south_east", "south_west",
                "north_west", "north_east"}) {
            rails.add(Block.RAIL.withProperty("shape", shape));
        }
        for (String shape : new String[]{"north_south", "east_west",
                "ascending_east", "ascending_west", "ascending_north",
                "ascending_south"}) {
            for (String powered : new String[]{"true", "false"}) {
                rails.add(Block.POWERED_RAIL.withProperty("shape", shape)
                        .withProperty("powered", powered));
                rails.add(Block.ACTIVATOR_RAIL.withProperty("shape", shape)
                        .withProperty("powered", powered));
            }
        }
        assertEquals(34, rails.size());

        MobTraversalProfile grounded = MobTraversalProfile.DEFAULT;
        MobTraversalProfile riding = MobTraversalProfile.builder("riding")
                .currentlyOnRail(true).build();
        for (Block rail : rails) {
            assertEquals(TerrainType.RAIL, TerrainClassifier.raw(rail),
                    () -> "not a rail: " + rail);
            assertEquals(TerrainType.UNPASSABLE_RAIL,
                    TerrainClassifier.transform(TerrainType.RAIL, grounded));
            assertNull(nodeAt(crossLane(rail, 1, 0.6, grounded), 3),
                    () -> "a grounded mob crossed " + rail);
            PathNode crossed = nodeAt(crossLane(rail, 1, 0.6, riding), 3);
            assertNotNull(crossed, () -> "a riding mob refused " + rail);
            assertEquals(1.0, crossed.y(), 1.0e-9);
        }
    }

    @Test
    void everyWallAndClosedGateStateRefusesTheLaneAndOpenGatesAllowIt() {
        int walls = 0;
        for (String north : new String[]{"none", "low", "tall"}) {
            for (String east : new String[]{"none", "low", "tall"}) {
                for (String up : new String[]{"true", "false"}) {
                    Block wall = Block.COBBLESTONE_WALL
                            .withProperty("north", north)
                            .withProperty("south", north)
                            .withProperty("east", east)
                            .withProperty("west", east)
                            .withProperty("up", up);
                    assertEquals(TerrainType.FENCE,
                            TerrainClassifier.raw(wall),
                            () -> "wall stopped being a fence: " + wall);
                    assertNull(nodeAt(crossLane(wall, 1, 0.6), 3),
                            () -> "walked through " + wall);
                    walls++;
                }
            }
        }
        assertEquals(18, walls);

        int gates = 0;
        for (String facing : FACINGS) {
            for (String inWall : new String[]{"false", "true"}) {
                Block closed = Block.OAK_FENCE_GATE
                        .withProperty("facing", facing)
                        .withProperty("in_wall", inWall)
                        .withProperty("open", "false");
                assertEquals(TerrainType.FENCE,
                        TerrainClassifier.raw(closed));
                assertNull(nodeAt(crossLane(closed, 1, 0.6), 3),
                        () -> "walked through " + closed);
                PathNode through = nodeAt(crossLane(
                        closed.withProperty("open", "true"), 1, 0.6), 3);
                assertNotNull(through,
                        () -> "refused an open gate facing " + facing);
                assertEquals(1.0, through.y(), 1.0e-9);
                gates++;
            }
        }
        assertEquals(8, gates);
    }

    @Test
    void ladderAnchorsHangOnTheFreeSideForAllFourFacings() {
        GroundCapabilities climbing = GroundCapabilities.STANDARD
                .withClimbables(ClimbableCapabilities.STANDARD);
        Map<String, int[]> free = Map.of(
                "north", new int[]{0, -1}, "south", new int[]{0, 1},
                "east", new int[]{1, 0}, "west", new int[]{-1, 0});
        for (String facing : FACINGS) {
            TestWorld world = ladderTower(facing, 4);
            PathResult result = pathfinder.findPath(world, new Pos(0.5 + free.get(facing)[0],
                            1, 0.5 + free.get(facing)[1]), new Pos(0.5, 4, 0.5), STANDARD, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(24).reachRange(0).maxVisitedMultiplier(8).build(), climbing, SearchControl.NONE);
            assertTrue(result.found(), () -> "no climb for " + facing);
            List<PathNode> climbs = result.nodes().stream().filter(node ->
                    node.movement() == PathNode.Movement.CLIMB).toList();
            assertFalse(climbs.isEmpty(), () -> "no climb edges: " + facing);
            for (PathNode node : climbs) {
                double dx = node.x() - (node.graphX() + 0.5);
                double dz = node.z() - (node.graphZ() + 0.5);
                assertTrue(Math.abs(dx) > 0.5 || Math.abs(dz) > 0.5,
                        () -> facing + " climb waypoint stayed inside the "
                                + "ladder cell: " + node);
                assertTrue(dx * free.get(facing)[0]
                                + dz * free.get(facing)[1] > 0,
                        () -> facing + " climb waypoint hung on the panel "
                                + "side: " + node);
            }
        }
    }

    private static List<Block> stairStates() {
        List<Block> out = new ArrayList<>();
        for (String waterlogged : new String[]{"false", "true"}) {
            for (String facing : FACINGS) {
                for (String half : new String[]{"bottom", "top"}) {
                    for (String shape : SHAPES) {
                        out.add(Block.OAK_STAIRS
                                .withProperty("facing", facing)
                                .withProperty("half", half)
                                .withProperty("shape", shape)
                                .withProperty("waterlogged", waterlogged));
                    }
                }
            }
        }
        return out;
    }

    private static double surface(Block block) {
        return block.collisionShape().relativeEnd().y();
    }

    private static PathNode nodeAt(PathResult result, int graphX) {
        return result.nodes().stream()
                .filter(node -> node.graphX() == graphX)
                .findFirst().orElse(null);
    }

    private PathResult crossLane(Block fixture, int dy, double width) {
        return crossLane(fixture, dy, width, MobTraversalProfile.DEFAULT);
    }

    private PathResult crossLane(Block fixture, int dy, double width,
                                 MobTraversalProfile profile) {
        return pathfinder.findPath(corridor(fixture, dy), new Pos(0.5, 1, 0.5), new Pos(5.5, 1, 0.5), new BoundingBox(width, 1.95, width), profile, GroundSearchLimits.builder().maxPathLength(32).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
    }

    /** Sealed one-wide corridor: a refused cell has no detour around it. */
    private static TestWorld corridor(Block fixture, int dy) {
        TestWorld world = new TestWorld();
        for (int x = -2; x <= 8; x++) {
            world.set(x, 0, 0, Block.STONE);
            world.column(x, -1, 0, 5, Block.STONE);
            world.column(x, 1, 0, 5, Block.STONE);
        }
        for (int y = 0; y <= 5; y++) {
            world.set(-1, y, 0, Block.STONE);
            world.set(7, y, 0, Block.STONE);
        }
        world.set(3, dy, 0, fixture);
        return world;
    }

    private static TestWorld ladderTower(String facing, int top) {
        TestWorld world = new TestWorld().floor(-2, 2, -2, 2, 0, Block.STONE);
        int backX = switch (facing) {
            case "east" -> -1;
            case "west" -> 1;
            default -> 0;
        };
        int backZ = switch (facing) {
            case "north" -> 1;
            case "south" -> -1;
            default -> 0;
        };
        for (int y = 1; y <= top; y++) {
            world.set(0, y, 0, Block.LADDER.withProperty("facing", facing));
            world.set(backX, y, backZ, Block.STONE);
        }
        return world;
    }
}
