package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.OpenableBlockFamily;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Forced-corridor differentials for stateful door and rail transformations.
 */
class DoorRailTraversalTest {
    private static final BoundingBox MOB = new BoundingBox(0.6, 1.8, 0.6);
    private static final Pos START = new Pos(0.5, 1, 0.5);
    private final EntityPathfinder pathfinding = new EntityPathfinder();

    @Test
    void villagerPassesClosedWoodDoorButZombieAndIronDoorDoNot() {
        TestWorld wooden = corridor();
        installDoor(wooden, 2, Block.OAK_DOOR);
        TestWorld iron = corridor();
        installDoor(iron, 2, Block.IRON_DOOR);

        PathResult villagerWood = find(
                wooden, EntityType.VILLAGER, new Pos(0.5, 1, 0.5));
        PathResult zombieWood = find(
                wooden, EntityType.ZOMBIE, new Pos(0.5, 1, 0.5));
        PathResult villagerIron = find(
                iron, EntityType.VILLAGER, new Pos(0.5, 1, 0.5));

        assertTrue(villagerWood.found(), villagerWood::toString);
        assertEquals(List.of(0, 1, 2, 3, 4),
                villagerWood.nodes().stream().map(PathNode::graphX).toList());
        assertFalse(zombieWood.found(),
                "closed wood door is negative without both open/pass flags");
        assertFalse(villagerIron.found(),
                "canOpenDoors transforms only hand-openable wooden doors");
    }

    @Test
    void railIsUnpassableOffRailAndPassableWhenRequestStartsOnRail() {
        TestWorld isolatedRail = corridor();
        isolatedRail.set(2, 1, 0, Block.RAIL);
        PathResult approaching = find(
                isolatedRail, EntityType.ZOMBIE, new Pos(0.5, 1, 0.5));

        TestWorld railLine = corridor();
        for (int x = 0; x <= 4; x++) railLine.set(x, 1, 0, Block.RAIL);
        PathResult alreadyOnRail = find(
                railLine, EntityType.ZOMBIE, new Pos(0.5, 1, 0.5));

        assertFalse(approaching.found(),
                "request snapshot transforms RAIL to UNPASSABLE_RAIL off-rail");
        assertTrue(alreadyOnRail.found(), alreadyOnRail::toString);
        assertTrue(alreadyOnRail.nodes().stream().allMatch(
                node -> node.graphZ() == 0 && node.graphY() == 1));
        assertEquals(5, alreadyOnRail.nodes().size());
    }

    @Test
    void closedFenceGateOpensARouteOnlyForAConfiguredManipulator() {
        TestWorld world = corridor().set(2, 1, 0, Block.OAK_FENCE_GATE
                .withProperty("facing", "north")
                .withProperty("open", "false"));

        PathResult ordinary = find(world, zombie(), START);
        PathResult manipulating = find(world, manipulating(
                BlockManipulationCapabilities.of(
                        OpenableBlockFamily.FENCE_GATE)), START);

        assertFalse(ordinary.found(),
                "baseline prices a closed fence gate as an impassable FENCE");
        assertTrue(manipulating.found(), manipulating::toString);
        assertEquals(List.of(0, 1, 2, 3, 4),
                manipulating.nodes().stream().map(PathNode::graphX).toList());
    }

    @Test
    void trapdoorFamilyIsIndependentAndExcludesIronTrapdoors() {
        TestWorld wooden = corridor().set(2, 1, 0, trapdoor(Block.OAK_TRAPDOOR));
        TestWorld iron = corridor().set(2, 1, 0, trapdoor(Block.IRON_TRAPDOOR));

        PathResult doorsOnly = find(wooden,
                trapdoorAverse(BlockManipulationCapabilities.STANDARD), START);
        PathResult withTrapdoors = find(wooden, trapdoorAverse(
                BlockManipulationCapabilities.of(
                        OpenableBlockFamily.TRAPDOOR)), START);
        PathResult ironWithTrapdoors = find(iron, trapdoorAverse(
                BlockManipulationCapabilities.of(
                        OpenableBlockFamily.TRAPDOOR)), START);

        assertFalse(doorsOnly.found(),
                "a door permission must not cover trapdoors");
        assertTrue(withTrapdoors.found(), withTrapdoors::toString);
        assertEquals(List.of(0, 1, 2, 3, 4),
                withTrapdoors.nodes().stream().map(PathNode::graphX).toList());
        assertFalse(ironWithTrapdoors.found(),
                "iron trapdoors cannot be opened by hand");
    }

    @Test
    void ironDoorStaysImpassableForEveryConfiguredFamily() {
        TestWorld iron = corridor();
        installDoor(iron, 2, Block.IRON_DOOR);

        PathResult result = find(iron, manipulating(
                BlockManipulationCapabilities.of(
                        OpenableBlockFamily.values())), START);

        assertFalse(result.found(), "iron doors are never hand-openable");
    }

    private PathResult find(TestWorld world, EntityType type, Pos start) {
        return find(world, BuiltinNavigationProfiles.forEntityType(type), start);
    }

    private PathResult find(TestWorld world, NavigationProfile profile,
                            Pos start) {
        return pathfinding.findPath(NavigationRequest.builder(
                        world, start, new Pos(4.5, 1, 0.5), MOB, profile)
                .maxPathLength(16)
                .maxVisitedMultiplier(8)
                .build(), SearchControl.NONE);
    }

    private static NavigationProfile zombie() {
        return BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
    }

    private static NavigationProfile manipulating(
            BlockManipulationCapabilities capabilities) {
        NavigationProfile base = zombie();
        return base.withMobProfile(MobTraversalProfile.builder("manipulator")
                .from(base.mobProfile())
                .blockManipulation(capabilities)
                .build());
    }

    /** A closed trapdoor is otherwise a free-to-enter thin slab. */
    private static NavigationProfile trapdoorAverse(
            BlockManipulationCapabilities capabilities) {
        NavigationProfile base = zombie();
        return base.withMobProfile(MobTraversalProfile.builder("trapdoor_averse")
                .from(base.mobProfile())
                .malus(TerrainType.TRAPDOOR, -1)
                .blockManipulation(capabilities)
                .build());
    }

    /**
     * Faced across the corridor, so the panel it swings to when opened stands
     * along the lane rather than over it and the mob still fits past.
     */
    private static Block trapdoor(Block block) {
        return block.withProperty("facing", "north")
                .withProperty("half", "bottom")
                .withProperty("open", "false");
    }

    private static TestWorld corridor() {
        TestWorld world = new TestWorld().floor(-2, 6, -1, 1, 0, Block.STONE);
        for (int x = -2; x <= 6; x++) {
            world.column(x, -1, 1, 4, Block.STONE);
            world.column(x, 1, 1, 4, Block.STONE);
        }
        return world;
    }

    private static void installDoor(TestWorld world, int x, Block door) {
        Block base = door.withProperty("facing", "east")
                .withProperty("hinge", "left")
                .withProperty("open", "false");
        world.set(x, 1, 0, base.withProperty("half", "lower"));
        world.set(x, 2, 0, base.withProperty("half", "upper"));
    }
}
