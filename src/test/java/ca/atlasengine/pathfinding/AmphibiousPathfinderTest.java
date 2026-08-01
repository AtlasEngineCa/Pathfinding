package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathNodeCost;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.EntityTraversalState;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.influence.NavigationZoneInfluence;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class AmphibiousPathfinderTest {
    private static final BoundingBox FROG_BOX = new BoundingBox(0.5, 0.5, 0.5);
    private final EntityPathfinder pathfinding = new EntityPathfinder();

    @Test
    void registryMatchesDirectAmphibiousUsers() {
        assertEquals(NavigationMode.AMPHIBIOUS,
                BuiltinNavigationProfiles.forEntityType(EntityType.AXOLOTL).mode());
        assertEquals(NavigationMode.AMPHIBIOUS,
                BuiltinNavigationProfiles.forEntityType(EntityType.DROWNED).mode());
        assertEquals(NavigationMode.AMPHIBIOUS,
                BuiltinNavigationProfiles.forEntityType(EntityType.TURTLE).mode());
        assertTrue(BuiltinNavigationProfiles.forEntityType(EntityType.FROG)
                .prefersShallowWater());
        assertFalse(BuiltinNavigationProfiles.forEntityType(EntityType.AXOLOTL)
                .prefersShallowWater());
    }

    @Test
    void crossesFromLandThroughWaterAndBackToLand() {
        TestWorld world = new TestWorld().floor(-2, 10, -3, 3, 0, Block.STONE);
        for (int x = 2; x <= 6; x++) {
            for (int z = -2; z <= 2; z++) {
                world.set(x, 0, z, Block.WATER);
                world.set(x, 1, z, Block.WATER);
            }
        }

        PathResult result = find(world, new Vec(0.5, 1, 0.5),
                new Vec(8.5, 1, 0.5), EntityType.AXOLOTL, List.of());

        assertTrue(result.found(), result::toString);
        assertTrue(result.nodes().stream().anyMatch(node ->
                node.blockX() >= 2 && node.blockX() <= 6));
        assertEquals(8, result.nodes().getLast().blockX());
    }

    @Test
    void liveSearchChargesFourForAWaterBorderCell() {
        TestWorld corridor = new TestWorld();
        for (int x = 0; x <= 6; x++) corridor.set(x, 0, 0, Block.WATER);
        corridor.set(3, 1, 0, Block.STONE);

        PathResult result = find(corridor, new Vec(0.5, 0.5, 0.5),
                new Vec(6.5, 0, 0.5), EntityType.AXOLOTL, List.of());

        assertTrue(result.found(), result::toString);
        PathNodeCost border = result.costAt(3, 0, 0);
        assertNotNull(border, result::toString);
        assertEquals(TerrainType.WATER_BORDER, border.terrain());
        assertEquals(4, border.malus());
        assertEquals(0, result.costAt(2, 0, 0).malus());
    }

    @Test
    void dynamicAvoidanceStillComposesWithMixedMediumSearch() {
        TestWorld world = new TestWorld().floor(-2, 10, -4, 4, 0, Block.STONE);
        for (int x = 2; x <= 6; x++) {
            for (int z = -3; z <= 3; z++) {
                world.set(x, 0, z, Block.WATER);
                world.set(x, 1, z, Block.WATER);
            }
        }
        NavigationZoneInfluence danger = new NavigationZoneInfluence(
                new Vec(3, 0, -1), new Vec(5, 3, 1),
                true, 0, "predator");

        PathResult result = find(world, new Vec(0.5, 1, 0.5),
                new Vec(8.5, 1, 0.5), EntityType.FROG, List.of(danger));

        assertTrue(result.found(), result::toString);
        assertTrue(result.nodes().stream().noneMatch(node ->
                node.x() >= 3 && node.x() <= 5
                        && node.z() >= -1 && node.z() <= 1));
    }

    @Test
    void shallowWaterPreferenceChargesOneBelowSeaLevelMinusTen() {
        TestWorld column = new TestWorld().column(0, 0, 40, 56, Block.WATER);

        PathResult frog = find(column, new Vec(0.5, 55.5, 0.5),
                new Vec(0.5, 41.5, 0.5), EntityType.FROG, List.of());
        PathResult axolotl = find(column, new Vec(0.5, 55.5, 0.5),
                new Vec(0.5, 41.5, 0.5), EntityType.AXOLOTL, List.of());

        assertTrue(frog.found(), frog::toString);
        assertEquals(4, MobTraversalProfile.FROG.malus(TerrainType.WATER));
        assertEquals(1, frog.costAt(0, 52, 0).malus());
        assertEquals(0, frog.costAt(0, 53, 0).malus());
        assertTrue(axolotl.found(), axolotl::toString);
        assertEquals(0, axolotl.costAt(0, 52, 0).malus());
    }

    @Test
    void amphibiousSearchRejectsVerticalNeighborsIntoWaterBorder() {
        PathResult open = find(waterChimney(), new Vec(0.5, 0.5, 0.5),
                new Vec(0.5, 4.5, 0.5), EntityType.AXOLOTL, List.of());
        assertTrue(open.found(), open::toString);
        assertEquals(5, open.nodes().getLast().blockY());
        assertTrue(open.nodes().stream()
                .anyMatch(node -> node.movement() == PathNode.Movement.SWIM));

        TestWorld border = waterChimney();
        border.set(1, 3, 0, Block.STONE);
        PathResult blocked = find(border, new Vec(0.5, 0.5, 0.5),
                new Vec(0.5, 4.5, 0.5), EntityType.AXOLOTL, List.of());
        assertFalse(blocked.found(), blocked::toString);
        assertTrue(blocked.nodes().stream().allMatch(node -> node.blockY() <= 2),
                blocked::toString);
    }

    @Test
    void searchDetoursAroundCostlyWaterBorderCells() {
        PathResult open = find(waterSlab(), new Vec(0.5, 0.5, 0.5),
                new Vec(6.5, 0, 0.5), EntityType.AXOLOTL, List.of());
        assertTrue(open.found(), open::toString);
        assertTrue(open.nodes().stream().anyMatch(node ->
                node.blockX() == 3 && node.blockZ() == 0), open::toString);

        TestWorld border = waterSlab();
        border.set(3, 1, 0, Block.STONE);
        PathResult detour = find(border, new Vec(0.5, 0.5, 0.5),
                new Vec(6.5, 0, 0.5), EntityType.AXOLOTL, List.of());
        assertTrue(detour.found(), detour::toString);
        assertTrue(detour.nodes().stream().noneMatch(node ->
                node.blockX() == 3 && node.blockZ() == 0), detour::toString);
    }

    @Test
    void shallowWaterPreferenceRaisesFrogAboveDeepWater() {
        PathResult frog = find(forkedPool(), new Vec(0.5, 53.5, 0.5),
                new Vec(8.5, 53.5, 0.5), EntityType.FROG, List.of());
        PathResult axolotl = find(forkedPool(), new Vec(0.5, 53.5, 0.5),
                new Vec(8.5, 53.5, 0.5), EntityType.AXOLOTL, List.of());

        assertTrue(frog.found(), frog::toString);
        assertTrue(frog.nodes().stream().allMatch(node -> node.blockY() >= 53),
                frog::toString);
        assertTrue(axolotl.found(), axolotl::toString);
        assertTrue(axolotl.nodes().stream().anyMatch(node -> node.blockY() < 53),
                axolotl::toString);
    }

    @Test
    void liveAmphibiousSearchLeavesCallerProfileUnchanged() {
        MobTraversalProfile frog = BuiltinNavigationProfiles
                .forEntityType(EntityType.FROG).mobProfile();
        assertSame(MobTraversalProfile.FROG, frog);
        assertEquals(4, frog.malus(TerrainType.WATER));
        assertEquals(0, frog.malus(TerrainType.WALKABLE));
        assertEquals(0, frog.malus(TerrainType.WATER_BORDER));

        PathResult result = find(waterChimney(), new Vec(0.5, 0.5, 0.5),
                new Vec(0.5, 4.5, 0.5), EntityType.FROG, List.of());

        assertTrue(result.found(), result::toString);
        assertEquals(4, frog.malus(TerrainType.WATER),
                "search-local WATER malus must not leak into the mob profile");
        assertEquals(0, frog.malus(TerrainType.WALKABLE),
                "search-local WALKABLE malus must not leak into the mob profile");
        assertEquals(0, frog.malus(TerrainType.WATER_BORDER),
                "search-local WATER_BORDER malus must not leak into the mob profile");
    }

    @Test
    void waterStartAndTargetUseAmphibiousYRounding() {
        TestWorld water = waterCube(-2, 3, -2, 5, -2, 2);
        PathResult axolotl = find(water, new Vec(0.5, 0.5, 0.5),
                new Vec(2.5, 2.6, 0.5), EntityType.AXOLOTL, List.of());
        PathResult frog = find(water, new Vec(0.5, 0.5, 0.5),
                new Vec(2.5, 2.6, 0.5), EntityType.FROG, List.of());

        assertTrue(axolotl.found(), axolotl::toString);
        assertEquals(1, axolotl.nodes().getFirst().blockY(),
                "base amphibious start is floor(minY + 0.5)");
        assertEquals(3, axolotl.nodes().getLast().blockY(),
                "amphibious target is floor(y + 0.5)");
        assertTrue(frog.found(), frog::toString);
        assertEquals(0, frog.nodes().getFirst().blockY(),
                "frog overrides the water start to floor(minY)");
    }

    private PathResult find(TestWorld world, Vec start, Vec target,
                            EntityType type, List<NavigationInfluence> influences) {
        return pathfinding.findPath(NavigationRequest.builder(
                        world, start, target, FROG_BOX,
                        BuiltinNavigationProfiles.forEntityType(type))
                .maxPathLength(40)
                .maxVisitedMultiplier(8)
                .influences(influences)
                .entityState(new EntityTraversalState(
                        true,
                        world.getBlock(start).key().value().equals("water"),
                        java.util.Set.of(), -64))
                .build(), SearchControl.NONE);
    }

    private static TestWorld waterCube(int minX, int maxX, int minY, int maxY,
                                       int minZ, int maxZ) {
        TestWorld world = new TestWorld();
        for (int x = minX; x <= maxX; x++) {
            for (int y = minY; y <= maxY; y++) {
                for (int z = minZ; z <= maxZ; z++) {
                    world.set(x, y, z, Block.WATER);
                }
            }
        }
        return world;
    }

    private static TestWorld waterChimney() {
        return waterCube(-2, 2, 0, 2, -2, 2)
                .column(0, 0, 3, 5, Block.WATER);
    }

    private static TestWorld waterSlab() {
        TestWorld world = new TestWorld();
        for (int x = 0; x <= 6; x++) {
            for (int z = -1; z <= 1; z++) world.set(x, 0, z, Block.WATER);
        }
        return world;
    }

    private static TestWorld forkedPool() {
        TestWorld world = new TestWorld();
        world.column(0, 0, 51, 57, Block.WATER);
        world.column(8, 0, 51, 57, Block.WATER);
        for (int x = 0; x <= 8; x++) {
            world.set(x, 51, 0, Block.WATER);
            world.set(x, 57, 0, Block.WATER);
        }
        return world;
    }
}
