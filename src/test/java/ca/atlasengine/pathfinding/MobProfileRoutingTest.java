package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Per-profile terrain maluses observed through a real route rather than
 * through the profile table. The same lane and the same hazard produce
 * different routes purely because the mob prices the hazard differently.
 */
class MobProfileRoutingTest {
    private static final BoundingBox MOB = new BoundingBox(0.6, 1.8, 0.6);
    private final DiscreteGroundPathfinder pathfinder =
            new DiscreteGroundPathfinder();

    /** A fire cell is forbidden to an animal and merely expensive to a warden. */
    @Test
    void animalAvoidsFireWhileWardenMayCrossIt() {
        assertEquals(-1, MobTraversalProfile.ANIMAL.malus(TerrainType.FIRE));
        assertTrue(MobTraversalProfile.WARDEN.malus(TerrainType.FIRE) >= 0);

        PathResult animal = lane(lane().set(2, 1, 0, Block.FIRE),
                MobTraversalProfile.ANIMAL);
        PathResult warden = lane(lane().set(2, 1, 0, Block.FIRE),
                MobTraversalProfile.WARDEN);

        assertTrue(animal.found(), animal::toString);
        assertTrue(animal.nodes().stream().noneMatch(node ->
                        node.graphX() == 2 && node.graphZ() == 0),
                () -> "animal walked into fire: " + animal.nodes());
        assertTrue(warden.found(), warden::toString);
        assertTrue(warden.nodes().stream().anyMatch(node ->
                        node.graphX() == 2 && node.graphZ() == 0),
                () -> "warden refused a crossable fire cell: "
                        + warden.nodes());
    }

    /**
     * A cactus is DAMAGING. The default profile forbids it; the fox override
     * accepts it and uses the direct lane.
     */
    @Test
    void foxAcceptsDamagingCactusWhileDefaultRoutesAround() {
        assertEquals(-1, MobTraversalProfile.DEFAULT.malus(TerrainType.DAMAGING));
        assertTrue(MobTraversalProfile.FOX.malus(TerrainType.DAMAGING) >= 0);

        PathResult normal = lane(lane().set(2, 1, 0, Block.CACTUS),
                MobTraversalProfile.DEFAULT);
        PathResult fox = lane(lane().set(2, 1, 0, Block.CACTUS),
                MobTraversalProfile.FOX);

        assertTrue(normal.found(), normal::toString);
        assertTrue(normal.nodes().stream().noneMatch(node ->
                        node.graphX() == 2 && node.graphZ() == 0),
                () -> "default profile entered a cactus: " + normal.nodes());
        assertTrue(fox.found(), fox::toString);
        assertTrue(fox.nodes().stream().anyMatch(node ->
                        node.graphX() == 2 && node.graphZ() == 0),
                () -> "fox override did not permit the damaging node: "
                        + fox.nodes());
        assertTrue(fox.nodes().stream().allMatch(node -> node.graphZ() == 0),
                () -> "fox took a detour it had no reason to take: "
                        + fox.nodes());
    }

    /**
     * One flooded lane, three answers: the default profile pays the water
     * malus and prefers a dry detour, a water animal swims straight through,
     * and an enderman refuses every flooded cell.
     */
    @Test
    void waterMalusChangesRouteByEntityProfile() {
        PathResult normal = lane(floodedLane(), MobTraversalProfile.DEFAULT);
        PathResult fish = lane(floodedLane(), MobTraversalProfile.WATER_ANIMAL);
        PathResult enderman = lane(floodedLane(), MobTraversalProfile.ENDERMAN);

        assertTrue(normal.found(), normal::toString);
        assertTrue(normal.nodes().stream().anyMatch(node -> node.graphZ() != 0),
                () -> "default water malus should favour a dry detour: "
                        + normal.nodes());
        assertTrue(fish.found(), fish::toString);
        assertTrue(fish.nodes().stream().allMatch(node -> node.graphZ() == 0),
                () -> "water animal should take the direct water route: "
                        + fish.nodes());
        assertTrue(enderman.found(), enderman::toString);
        assertTrue(enderman.nodes().stream().noneMatch(node ->
                        node.graphZ() == 0 && node.graphX() >= 1
                                && node.graphX() <= 4),
                () -> "enderman entered forbidden water: " + enderman.nodes());
    }

    /**
     * A forbidden terrain type is a neighbour filter, not a node filter.
     * A mob standing on ordinary ground may not step into water it prices at
     * -1, while a mob whose own node is already forbidden keeps expanding:
     * the neighbour rule is {@code neighbour >= 0 || current < 0}, so a start
     * inside forbidden terrain is never a dead end.
     */
    @Test
    void forbiddenWaterBlocksEntryFromDryLandButNotFromInsideIt() {
        assertEquals(-1, MobTraversalProfile.ENDERMAN.malus(TerrainType.WATER));
        assertTrue(MobTraversalProfile.ENDERMAN.canFloat());

        PathResult fromDryLand = pathfinder.findPath(floodedLane(), new Pos(0.5, 1, 0.5), new Pos(5.5, 1, 0.5), MOB, MobTraversalProfile.ENDERMAN, GroundSearchLimits.builder().maxPathLength(20).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
        PathResult fromInsideWater = pathfinder.findPath(floodedLane(), new Pos(1.5, 1, 0.5), new Pos(5.5, 1, 0.5), MOB, MobTraversalProfile.ENDERMAN, GroundSearchLimits.builder().maxPathLength(20).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);

        assertTrue(fromDryLand.found(), fromDryLand::toString);
        assertTrue(fromDryLand.nodes().stream().noneMatch(node ->
                        node.graphZ() == 0 && node.graphX() >= 1
                                && node.graphX() <= 4),
                () -> "a walkable node expanded into forbidden water: "
                        + fromDryLand.nodes());

        assertFalse(fromInsideWater.nodes().isEmpty(),
                "a start inside forbidden terrain still produces a start node");
        assertEquals(1, fromInsideWater.nodes().getFirst().graphX());
        assertTrue(fromInsideWater.found(), fromInsideWater::toString);
        assertTrue(fromInsideWater.nodes().stream().anyMatch(node ->
                        node.graphZ() == 0 && node.graphX() >= 2
                                && node.graphX() <= 4),
                () -> "a forbidden start node must still expand: "
                        + fromInsideWater.nodes());
    }

    private PathResult lane(TestWorld world, MobTraversalProfile profile) {
        return pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(5.5, 1, 0.5), MOB, profile, GroundSearchLimits.builder().maxPathLength(20).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
    }

    private static TestWorld lane() {
        return new TestWorld().floor(-2, 8, -3, 3, 0, Block.STONE);
    }

    private static TestWorld floodedLane() {
        TestWorld world = lane();
        for (int x = 1; x <= 4; x++) world.set(x, 1, 0, Block.WATER);
        return world;
    }
}
