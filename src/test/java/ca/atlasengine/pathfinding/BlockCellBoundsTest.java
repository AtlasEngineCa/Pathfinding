package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.influence.AllowedNavigationAreas;
import ca.atlasengine.pathfinding.influence.NavigationArea;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.influence.NavigationZoneInfluence;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Zone and area bounds are continuous, so a volume built straight from the
 * block coordinates it means to cover silently misses one whole cell per axis:
 * the mob walks through the far edge of a zone that looks configured, and is
 * shut out of the far edge of a road that looks wide enough. The named block
 * factories exist so that mistake cannot be expressed.
 */
class BlockCellBoundsTest {
    private static final BoundingBox BOX = new BoundingBox(0.6, 1.8, 0.6);
    private static final NavigationProfile ZOMBIE =
            BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
    private static final int FIRST = 2;
    private static final int LAST = 5;

    @Test
    void blockZoneForbidsEveryNamedCellIncludingTheMaximumOne() {
        NavigationZoneInfluence zone = NavigationZoneInfluence.blocks(
                new Vec(FIRST, 1, FIRST), new Vec(LAST, 2, LAST),
                true, 0, "closed");
        TestWorld world = new TestWorld();

        for (int x = FIRST; x <= LAST; x++) {
            for (int z = FIRST; z <= LAST; z++) {
                int cellX = x;
                int cellZ = z;
                assertTrue(zone.evaluate(world, standing(x, z), BOX).blocked(),
                        () -> "cell " + cellX + "," + cellZ + " was named but "
                                + "a mob standing in it is not excluded");
            }
        }
        assertFalse(zone.evaluate(world, standing(LAST + 1, LAST), BOX)
                        .blocked(),
                "a cell past the named maximum must stay open");
        assertFalse(zone.evaluate(world, standing(FIRST - 1, FIRST), BOX)
                        .blocked(),
                "a cell before the named minimum must stay open");
    }

    /** Corners in any order name the same cells. */
    @Test
    void blockZoneCornersAreOrderIndependent() {
        assertEquals(
                NavigationZoneInfluence.blocks(new Vec(FIRST, 1, FIRST),
                        new Vec(LAST, 2, LAST), true, 0, "closed"),
                NavigationZoneInfluence.blocks(new Vec(LAST, 2, LAST),
                        new Vec(FIRST, 1, FIRST), true, 0, "closed"));
    }

    /**
     * The continuous constructor keeps meaning exactly what it says, which is
     * also why building it from block coordinates is the trap: the maximum
     * cell of the range is left entirely walkable.
     */
    @Test
    void continuousZoneFromTheSameBlockCornersLeavesTheMaximumCellOpen() {
        NavigationZoneInfluence naive = new NavigationZoneInfluence(
                new Vec(FIRST, 1, FIRST), new Vec(LAST, 2, LAST),
                true, 0, "closed");

        assertFalse(naive.evaluate(new TestWorld(), standing(LAST, LAST), BOX)
                        .blocked(),
                "continuous bounds must not be quietly widened to cells");
    }

    @Test
    void blockAreaAdmitsAMobStandingInEveryNamedCell() {
        AllowedNavigationAreas road = AllowedNavigationAreas.of(
                NavigationArea.blocks(new Vec(FIRST, 1, FIRST),
                        new Vec(LAST, 2, LAST), "market-road"));
        TestWorld world = new TestWorld();

        for (int x = FIRST; x <= LAST; x++) {
            for (int z = FIRST; z <= LAST; z++) {
                int cellX = x;
                int cellZ = z;
                assertFalse(road.evaluate(world, standing(x, z), BOX).blocked(),
                        () -> "cell " + cellX + "," + cellZ + " was named but "
                                + "a mob standing in it is shut out");
            }
        }
        assertTrue(road.evaluate(world, standing(LAST + 1, LAST), BOX)
                        .blocked(),
                "a cell past the named maximum must stay outside the road");
    }

    /**
     * The same off-by-one seen through a route rather than a predicate: a wall
     * of block cells with one gap must be walked around, not through.
     */
    @Test
    void routeRefusesEveryCellOfABlockZoneAndUsesTheGap() {
        NavigationInfluence wall = NavigationZoneInfluence.blocks(
                new Vec(5, 1, -6), new Vec(5, 2, 2), true, 0, "wall");

        PathResult around = route(List.of(wall));
        PathResult through = route(List.of(new NavigationZoneInfluence(
                new Vec(5, 1, -6), new Vec(5, 2, 2), true, 0, "wall")));

        assertTrue(around.found(), around::toString);
        assertTrue(around.nodes().stream().noneMatch(node ->
                        node.graphX() == 5 && node.graphZ() <= 2),
                () -> "the route crossed a named wall cell: " + around.nodes());
        assertTrue(around.nodes().stream().anyMatch(node -> node.graphZ() >= 3),
                () -> "the route did not use the gap: " + around.nodes());
        assertEquals(9, around.nodes().getLast().graphX());

        assertTrue(through.found(), through::toString);
        assertTrue(through.nodes().stream().anyMatch(node ->
                        node.graphX() == 5 && node.graphZ() == 0),
                () -> "continuous bounds one short of the wall cannot stop a "
                        + "mob walking through it: " + through.nodes());
    }

    private static Vec standing(int blockX, int blockZ) {
        return new Vec(blockX + 0.5, 1, blockZ + 0.5);
    }

    private PathResult route(List<NavigationInfluence> influences) {
        TestWorld world = new TestWorld()
                .floor(-1, 10, -6, 6, 0, Block.STONE);
        return new EntityPathfinder().findPath(NavigationRequest.builder(
                                world, new Pos(0.5, 1, 0.5),
                                new Pos(9.5, 1, 0.5), BOX, ZOMBIE)
                        .maxPathLength(40)
                        .maxVisitedMultiplier(8)
                        .influences(influences)
                        .build(),
                SearchControl.NONE);
    }
}
