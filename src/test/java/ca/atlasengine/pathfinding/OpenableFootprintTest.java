package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.internal.JumpArc;
import ca.atlasengine.pathfinding.internal.movement.BlockManipulator;
import ca.atlasengine.pathfinding.internal.movement.ClimbMovementExecutor;
import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.OpenableBlockFamily;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * A mob opens every cell it stands over, not one of them.
 *
 * <p>A route waypoint is the centre of the entity box, so a footprint wider
 * or deeper than one cell covers cells the waypoint's own cell is not, and a
 * ladder-anchored waypoint covers none of them. The planner reads exactly the
 * cells the follower writes, which is what the scan below holds them to.</p>
 */
class OpenableFootprintTest {
    private static final BoundingBox NARROW = new BoundingBox(0.6, 1.95, 0.6);
    private static final BoundingBox WIDE = new BoundingBox(1.4, 1.95, 1.4);
    private static final String[] FACINGS =
            {"north", "south", "east", "west"};
    private static final double APEX = 1.2;

    @Test
    void aWideMobOpensBothCellsOfItsFootprint() {
        TestWorld world = new TestWorld();
        world.set(3, 1, 4, hatch("east"));
        world.set(4, 1, 4, hatch("east"));
        world.set(5, 1, 4, hatch("east"));

        new BlockManipulator().openAt(world, hatchOpener(),
                new Vec(4.0, 1, 4.0), WIDE);

        assertEquals("true", open(world, 3, 1, 4));
        assertEquals("true", open(world, 4, 1, 4),
                "the second footprint cell is the one a column view misses");
        assertEquals("false", open(world, 5, 1, 4),
                "nothing outside the box is touched");
    }

    @Test
    void aNarrowMobStillOpensOnlyTheCellItStandsIn() {
        TestWorld world = new TestWorld();
        for (int x = 3; x <= 5; x++) world.set(x, 1, 4, hatch("east"));

        new BlockManipulator().openAt(world, hatchOpener(),
                new Vec(4.5, 1, 4.5), NARROW);

        assertEquals("false", open(world, 3, 1, 4));
        assertEquals("true", open(world, 4, 1, 4));
        assertEquals("false", open(world, 5, 1, 4));
    }

    @Test
    void anOffsetBoxOpensItsActualCellsRatherThanACentredGuess() {
        TestWorld world = new TestWorld();
        for (int x = 3; x <= 6; x++) world.set(x, 1, 4, hatch("east"));
        BoundingBox offset = new BoundingBox(0.6, 1.95, 0.6,
                new Vec(1.1, 0, -0.3));

        new BlockManipulator().openAt(world, hatchOpener(),
                new Vec(4.5, 1, 4.5), offset);

        assertEquals("false", open(world, 4, 1, 4),
                "the waypoint cell is outside the offset box");
        assertEquals("true", open(world, 5, 1, 4));
        assertEquals("true", open(world, 6, 1, 4));
    }

    /**
     * A ladder anchor hangs the whole footprint outside the ladder's own
     * cell, so the waypoint's cell is not even one of the cells opened.
     */
    @Test
    void aLadderAnchoredWaypointOpensTheCellsTheBoxHangsOver() {
        TestWorld world = new TestWorld();
        world.set(0, 4, 0, hatch("east"));
        world.set(0, 4, -1, hatch("east"));
        world.set(1, 4, -1, hatch("east"));

        new BlockManipulator().openAt(world, hatchOpener(),
                new Vec(1.0, 4, -1.01), WIDE);

        assertEquals("false", open(world, 0, 4, 0),
                "the ladder column itself is behind the mob");
        assertEquals("true", open(world, 0, 4, -1));
        assertEquals("true", open(world, 1, 4, -1));
    }

    /**
     * A two-cell footprint reaches the hatch beside the ladder, so planning
     * the climb requires reading that hatch in the state the follower leaves
     * it in. The narrow control climbs the identical shaft either way.
     */
    @Test
    void aWideMobClimbsPastTheHatchItsFootprintOpens() {
        PathResult wide = climb(shaft(hatch("east")), WIDE).result();
        PathResult narrow = climb(shaft(hatch("east")), NARROW).result();

        assertTrue(wide.nodes().getLast().graphY() > 4,
                () -> "planned short of the hatch it opens: " + wide.nodes());
        assertTrue(narrow.nodes().getLast().graphY() > 4, narrow::toString);
    }

    /**
     * A landing platform paved with openables is one the follower opens
     * whole, so every cell of the footprint standing on it is a body cell
     * the search may leave to the swept arc.
     */
    @Test
    void aWideMobLandsOnAPlatformItsWholeFootprintOpens() {
        List<PathNode> wide = jump(gate("north"), WIDE).result().nodes();
        List<PathNode> narrow = jump(gate("north"), NARROW).result().nodes();

        assertTrue(wide.stream().anyMatch(
                        node -> node.movement() == PathNode.Movement.JUMP),
                () -> "no landing for a two-cell footprint: " + wide);
        assertTrue(narrow.stream().anyMatch(
                        node -> node.movement() == PathNode.Movement.JUMP),
                narrow::toString);
    }

    /**
     * The scan behind the planner-follower invariant. Every waypoint the
     * search emits has to survive the test the follower runs at it, once the
     * follower has opened what it opens, across every family, facing, half,
     * hinge, and both footprint sizes.
     */
    @Test
    void everyEmittedWaypointSurvivesTheFollowerThatOpensIt() {
        List<String> refused = new ArrayList<>();
        int emitted = 0;
        for (Block openable : openables()) {
            for (BoundingBox box : new BoundingBox[]{NARROW, WIDE}) {
                OpenableBlockFamily family =
                        BlockTraversalData.openableFamily(openable);
                emitted += follow(walk(openable, box), box, family,
                        openable, "walk", refused);
                emitted += follow(climb(shaft(openable), box), box, family,
                        openable, "climb", refused);
                emitted += follow(jump(openable, box), box, family,
                        openable, "jump", refused);
            }
        }

        assertEquals(List.of(), refused);
        assertTrue(emitted > 300,
                "the scan has to reach waypoints at all: " + emitted);
    }

    /** Replays the follower over one emitted route and counts refusals. */
    private int follow(Planned planned, BoundingBox box,
                       OpenableBlockFamily family, Block openable,
                       String kind, List<String> refused) {
        MobTraversalProfile profile = opener(family);
        BlockManipulator manipulator = new BlockManipulator();
        List<PathNode> nodes = planned.result.nodes();
        for (int index = 0; index < nodes.size(); index++) {
            PathNode node = nodes.get(index);
            manipulator.openAt(planned.world, profile, node.asVec(), box);
            if (index == 0) continue;
            PathNode previous = nodes.get(index - 1);
            boolean clear = switch (node.movement()) {
                case CLIMB -> ClimbMovementExecutor.movementClear(
                        planned.world, box, node.asVec());
                case JUMP -> JumpArc.clear(planned.world, box,
                        previous.asVec(), node.asVec(), APEX, null);
                default -> !insidePanel(planned.world, node.asVec(), box);
            };
            if (!clear) {
                refused.add(kind + " " + describe(openable) + " w"
                        + box.width() + " at " + node);
            }
        }
        return Math.max(0, nodes.size() - 1);
    }

    /** Whether the box at a waypoint stands inside an openable's shape. */
    private static boolean insidePanel(TestWorld world, Point position,
                                       BoundingBox box) {
        var iterator = box.getBlocks(position);
        while (iterator.hasNext()) {
            var mutable = iterator.next();
            Point cell = new Vec(mutable.blockX(), mutable.blockY(),
                    mutable.blockZ());
            Block block = world.getBlock(cell.blockX(), cell.blockY(),
                    cell.blockZ(), Block.Getter.Condition.TYPE);
            if (BlockTraversalData.openableFamily(block) == null) continue;
            if (block.collisionShape().intersectBox(
                    position.sub(cell), box)) return true;
        }
        return false;
    }

    private record Planned(TestWorld world, PathResult result) {
    }

    // ------------------------------------------------------------ scenarios

    /** Wall with a three-cell doorway a two-cell footprint can reach. */
    private Planned walk(Block openable, BoundingBox box) {
        TestWorld world = new TestWorld().floor(-2, 9, -4, 4, 0, Block.STONE);
        for (int z = -4; z <= 4; z++) {
            if (z >= -1 && z <= 1) continue;
            world.column(3, z, 1, 4, Block.STONE);
        }
        for (int z = -1; z <= 1; z++) install(world, 3, 1, z, openable);
        boolean wide = box.width() > 1;
        return new Planned(world, find(world,
                wide ? new Pos(1.0, 1, 0.0) : new Pos(0.5, 1, 0.5),
                wide ? new Pos(7.0, 1, 0.0) : new Pos(7.5, 1, 0.5),
                box, openable, GroundCapabilities.STANDARD));
    }

    /** Ladder backed to the south with the openable hung to its north. */
    private static TestWorld shaft(Block openable) {
        TestWorld world = new TestWorld().floor(-4, 4, -4, 4, 0, Block.STONE);
        world.column(0, 1, 1, 9, Block.STONE);
        Block ladder = Block.LADDER.withProperty("facing", "north");
        for (int y = 1; y <= 8; y++) world.set(0, y, 0, ladder);
        install(world, 0, 4, -1, openable);
        return world;
    }

    private Planned climb(TestWorld world, BoundingBox box) {
        return new Planned(world, find(world, new Pos(0.5, 1, -0.5),
                new Pos(0.5, 8, -0.5), box, firstOpenable(world),
                GroundCapabilities.STANDARD.withClimbables(
                        ClimbableCapabilities.STANDARD)));
    }

    /**
     * Two platforms with the openable standing on the landing. The gap is
     * one cell wider than the footprint, so the far side is only reachable
     * by an arc and never by walking the overhang across.
     */
    private Planned jump(Block openable, BoundingBox box) {
        TestWorld world = new TestWorld();
        boolean wide = box.width() > 1;
        int span = wide ? 1 : 0;
        int far = 4 + 2 * span;
        world.floor(0, span, 0, span, 0, Block.STONE);
        world.floor(far, far + span, 0, span, 0, Block.STONE);
        for (int x = far; x <= far + span; x++) {
            for (int z = 0; z <= span; z++) install(world, x, 1, z, openable);
        }
        double center = (span + 1) * 0.5;
        return new Planned(world, find(world,
                new Pos(center, 1, center),
                new Pos(far + center, 1, center),
                box, openable, GroundCapabilities.STANDARD.withPlatformJump(
                        PlatformJumpCapabilities.builder().maxHorizontalDistance(8).maxRise(0).maxDrop(0).apexClearance(APEX).build())));
    }

    // -------------------------------------------------------------- helpers

    private PathResult find(TestWorld world, Pos start, Pos target,
                            BoundingBox box, Block openable,
                            GroundCapabilities capabilities) {
        NavigationProfile profile = BuiltinNavigationProfiles
                .forEntityType(EntityType.ZOMBIE)
                .withMobProfile(opener(
                        BlockTraversalData.openableFamily(openable)))
                .withGroundCapabilities(capabilities);
        return new EntityPathfinder().findPath(NavigationRequest.builder(
                        world, start, target, box, profile)
                .maxPathLength(48).maxVisitedMultiplier(8).build(),
                SearchControl.NONE);
    }

    private static MobTraversalProfile opener(OpenableBlockFamily family) {
        return MobTraversalProfile.builder("footprint")
                .blockManipulation(BlockManipulationCapabilities.of(family))
                .build();
    }

    private static MobTraversalProfile hatchOpener() {
        return opener(OpenableBlockFamily.TRAPDOOR);
    }

    private static void install(TestWorld world, int x, int y, int z,
                                Block openable) {
        if (BlockTraversalData.isDoor(openable)) {
            world.set(x, y, z, openable.withProperty("half", "lower"));
            world.set(x, y + 1, z, openable.withProperty("half", "upper"));
        } else {
            world.set(x, y, z, openable);
        }
    }

    private static Block firstOpenable(TestWorld world) {
        Block block = world.getBlock(0, 4, -1, Block.Getter.Condition.TYPE);
        return BlockTraversalData.openableFamily(block) == null
                ? Block.OAK_TRAPDOOR : block;
    }

    private static List<Block> openables() {
        List<Block> list = new ArrayList<>();
        for (String facing : FACINGS) {
            for (String hinge : new String[]{"left", "right"}) {
                list.add(Block.OAK_DOOR.withProperty("facing", facing)
                        .withProperty("hinge", hinge)
                        .withProperty("open", "false"));
            }
            for (String half : new String[]{"bottom", "top"}) {
                list.add(Block.OAK_TRAPDOOR.withProperty("facing", facing)
                        .withProperty("half", half)
                        .withProperty("open", "false"));
            }
            list.add(Block.OAK_FENCE_GATE.withProperty("facing", facing)
                    .withProperty("open", "false"));
        }
        return list;
    }

    private static String describe(Block block) {
        return block.key().value() + "/" + block.getProperty("facing") + "/"
                + (block.getProperty("hinge") == null
                ? block.getProperty("half") : block.getProperty("hinge"));
    }

    private static Block gate(String facing) {
        return Block.OAK_FENCE_GATE.withProperty("facing", facing)
                .withProperty("open", "false");
    }

    private static Block hatch(String facing) {
        return Block.OAK_TRAPDOOR.withProperty("facing", facing)
                .withProperty("half", "bottom")
                .withProperty("open", "false");
    }

    private static String open(TestWorld world, int x, int y, int z) {
        return world.getBlock(x, y, z, Block.Getter.Condition.TYPE)
                .getProperty("open");
    }
}
