package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.internal.movement.ClimbMovementExecutor;
import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.OpenableBlockFamily;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Physical clearance for blocks a profile may open. Terrain classification
 * already routes through them as {@code WALKABLE_DOOR}; these pin that the
 * planner's swept-reach, head-room, and climb box tests stop seeing the closed
 * state too, so a routable node is also a node the entity fits in.
 */
class OpenableClearanceTest {
    private static final BoundingBox MOB = new BoundingBox(0.6, 1.8, 0.6);
    private static final BoundingBox SMALL = new BoundingBox(0.4, 1.0, 0.4);
    private static final Block CLOSED_TRAPDOOR = Block.OAK_TRAPDOOR
            .withProperty("facing", "east")
            .withProperty("half", "bottom")
            .withProperty("open", "false");
    private final EntityPathfinder pathfinding = new EntityPathfinder();

    /**
     * A fence start is one of the few nodes baseline expands with a partial
     * collision type, so it is what drives the swept reach test.
     */
    @Test
    void sweptReachTestIgnoresADoorTheProfileMayOpen() {
        TestWorld world = corridor(-2, 6).set(0, 1, 0, Block.OAK_FENCE);
        installDoor(world, 1);

        PathResult villager = find(world, profile(EntityType.VILLAGER), SMALL,
                new Pos(0.5, 1, 0.5), new Pos(4.5, 1, 0.5));
        PathResult zombie = find(world, profile(EntityType.ZOMBIE), SMALL,
                new Pos(0.5, 1, 0.5), new Pos(4.5, 1, 0.5));

        assertTrue(villager.found(), villager::toString);
        assertEquals(List.of(0, 1, 2, 3, 4),
                villager.nodes().stream().map(PathNode::graphX).toList());
        assertFalse(zombie.found(),
                "a closed wood door stays solid without the door family");
    }

    @Test
    void headroomTestIgnoresAHatchTheProfileMayOpen() {
        TestWorld world = corridor(-2, 8);
        for (int x = 1; x <= 8; x++) world.set(x, 1, 0, Block.STONE);
        world.set(0, 2, 0, CLOSED_TRAPDOOR);

        PathResult opener = find(world, manipulating(
                        OpenableBlockFamily.TRAPDOOR), MOB,
                new Pos(0.5, 1, 0.5), new Pos(4.5, 2, 0.5));
        PathResult plain = find(world, profile(EntityType.ZOMBIE), MOB,
                new Pos(0.5, 1, 0.5), new Pos(4.5, 2, 0.5));

        assertTrue(opener.found(), opener::toString);
        assertEquals(PathNode.Movement.STEP_UP,
                opener.nodes().get(1).movement());
        assertFalse(plain.found(),
                "the closed hatch overhead leaves no room to step up");
    }

    @Test
    void climbClearanceIgnoresAHatchTheProfileMayOpen() {
        TestWorld unobstructed = shaft();
        TestWorld obstructed = shaft().set(0, 4, -1, CLOSED_TRAPDOOR);
        NavigationProfile opener = climbing(manipulating(
                OpenableBlockFamily.TRAPDOOR));
        NavigationProfile plain = climbing(profile(EntityType.ZOMBIE));

        PathResult control = find(unobstructed, opener, MOB,
                new Pos(0.5, 1, 0.5), new Pos(0.5, 7, 0.5));
        PathResult climbed = find(obstructed, opener, MOB,
                new Pos(0.5, 1, 0.5), new Pos(0.5, 7, 0.5));
        PathResult blocked = find(obstructed, plain, MOB,
                new Pos(0.5, 1, 0.5), new Pos(0.5, 7, 0.5));

        assertTrue(control.found(), control::toString);
        assertTrue(climbed.found(), climbed::toString);
        assertEquals(7, climbed.nodes().getLast().graphY());
        assertFalse(blocked.found(),
                "the hatch juts into the shaft for everyone else");
        assertTrue(blocked.nodes().getLast().graphY() < 4, blocked::toString);
    }

    /**
     * The follower drives to the emitted waypoint and refuses any position
     * whose box meets a shape, having first opened that waypoint's own
     * column. So the planner has to clear the same box at the same point.
     */
    @Test
    void climbNodesAreClearedAtTheWaypointTheFollowerDrivesTo() {
        TestWorld world = shaft().set(0, 4, -1, hatch("east"));
        NavigationProfile opener = climbing(manipulating(
                OpenableBlockFamily.TRAPDOOR));

        PathResult result = find(world, opener, MOB,
                new Pos(0.5, 1, 0.5), new Pos(0.5, 7, 0.5));

        assertTrue(result.found(), result::toString);
        TestWorld opened = shaft().set(0, 4, -1, hatch("east"));
        int climbs = 0;
        for (PathNode node : result.nodes()) {
            openColumn(opened, node);
            if (node.movement() != PathNode.Movement.CLIMB) continue;
            climbs++;
            assertEquals(node.graphZ() - 0.51, node.z(), 1.0e-9,
                    "the ladder anchor holds the whole grid footprint out");
            assertTrue(ClimbMovementExecutor.movementClear(
                            opened, MOB, node.asVec()),
                    () -> "the follower refuses " + node);
        }
        assertEquals(6, climbs, result::toString);
    }

    /**
     * An open trapdoor is a 0.1875 panel filling its column's climb face, not
     * nothing. A node behind one is a node the follower would refuse, however
     * freely the profile may open the closed state.
     */
    @Test
    void aHatchWhoseOpenPanelFillsTheClimbColumnIsNotPlannedThrough() {
        BoundingBox deep = new BoundingBox(0.6, 1.8, 0.8);
        TestWorld facingIn = shaft().set(0, 4, -1, hatch("north"));
        TestWorld facingAside = shaft().set(0, 4, -1, hatch("east"));
        NavigationProfile opener = climbing(manipulating(
                OpenableBlockFamily.TRAPDOOR));

        PathResult blocked = find(facingIn, opener, deep,
                new Pos(0.5, 1, 0.5), new Pos(0.5, 7, 0.5));
        PathResult aside = find(facingAside, opener, deep,
                new Pos(0.5, 1, 0.5), new Pos(0.5, 7, 0.5));

        assertTrue(aside.found(), aside::toString);
        assertEquals(7, aside.nodes().getLast().graphY());
        assertFalse(blocked.found(),
                "the opened panel leaves the climb column too shallow");
        assertTrue(blocked.nodes().getLast().graphY() < 4, blocked::toString);
    }

    @Test
    void aProfileThatCannotPassDoorsKeepsTheClosedCollisionShape() {
        TestWorld world = corridor(-2, 8);
        for (int x = 1; x <= 8; x++) world.set(x, 1, 0, Block.STONE);
        world.set(0, 2, 0, CLOSED_TRAPDOOR);
        NavigationProfile shut = profile(EntityType.ZOMBIE);
        shut = shut.withMobProfile(MobTraversalProfile.builder("shut")
                .from(shut.mobProfile())
                .blockManipulation(BlockManipulationCapabilities.of(
                        OpenableBlockFamily.TRAPDOOR))
                .canPassDoors(false)
                .build());

        PathResult result = find(world, shut, MOB,
                new Pos(0.5, 1, 0.5), new Pos(4.5, 2, 0.5));

        assertFalse(result.found(), result::toString);
    }

    private PathResult find(TestWorld world, NavigationProfile profile,
                           BoundingBox box, Pos start, Pos target) {
        return pathfinding.findPath(NavigationRequest.builder(
                        world, start, target, box, profile)
                .maxPathLength(32)
                .maxVisitedMultiplier(8)
                .build(), SearchControl.NONE);
    }

    private static NavigationProfile profile(EntityType type) {
        return BuiltinNavigationProfiles.forEntityType(type);
    }

    private static NavigationProfile manipulating(
            OpenableBlockFamily... families) {
        NavigationProfile base = profile(EntityType.ZOMBIE);
        return base.withMobProfile(MobTraversalProfile.builder("manipulator")
                .from(base.mobProfile())
                .blockManipulation(BlockManipulationCapabilities.of(families))
                .build());
    }

    private static NavigationProfile climbing(NavigationProfile profile) {
        return profile.withGroundCapabilities(profile.groundCapabilities()
                .withClimbables(ClimbableCapabilities.STANDARD));
    }

    private static TestWorld corridor(int minX, int maxX) {
        TestWorld world = new TestWorld().floor(minX, maxX, -1, 1, 0,
                Block.STONE);
        for (int x = minX; x <= maxX; x++) {
            world.column(x, -1, 1, 4, Block.STONE);
            world.column(x, 1, 1, 4, Block.STONE);
        }
        return world;
    }

    private static void installDoor(TestWorld world, int x) {
        Block door = Block.OAK_DOOR.withProperty("facing", "east")
                .withProperty("hinge", "left")
                .withProperty("open", "false");
        world.set(x, 1, 0, door.withProperty("half", "lower"));
        world.set(x, 2, 0, door.withProperty("half", "upper"));
    }

    private static Block hatch(String facing) {
        return CLOSED_TRAPDOOR.withProperty("facing", facing);
    }

    /** What {@code BlockManipulator.openAt} does at one hatch-opening node. */
    private static void openColumn(TestWorld world, PathNode node) {
        Point waypoint = node.asVec();
        int maxY = waypoint.blockY() + (int) Math.ceil(MOB.height());
        for (int y = waypoint.blockY(); y <= maxY; y++) {
            Block block = world.getBlock(waypoint.blockX(), y,
                    waypoint.blockZ(), Block.Getter.Condition.TYPE);
            if (BlockTraversalData.isTrapdoor(block)) {
                world.set(waypoint.blockX(), y, waypoint.blockZ(),
                        block.withProperty("open", "true"));
            }
        }
    }

    /** Ladder column backed by a wall to its south, open to the north. */
    private static TestWorld shaft() {
        TestWorld world = new TestWorld().floor(-2, 2, -2, 2, 0, Block.STONE);
        world.column(0, 1, 1, 8, Block.STONE);
        Block ladder = Block.LADDER.withProperty("facing", "north");
        for (int y = 1; y <= 7; y++) world.set(0, y, 0, ladder);
        return world;
    }
}
