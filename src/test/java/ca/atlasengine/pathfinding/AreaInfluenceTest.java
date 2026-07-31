package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.adaptive.SharedMeshNavigation;
import ca.atlasengine.pathfinding.influence.AreaInfluence;
import ca.atlasengine.pathfinding.influence.InfluenceResult;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import net.minestom.server.Tickable;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Area;
import net.minestom.server.coordinate.BlockVec;
import net.minestom.server.coordinate.Vec;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Minestom areas as navigation influences, including the shapes an
 * axis-aligned box cannot express.
 */
class AreaInfluenceTest {
    private static final BoundingBox BOX = new BoundingBox(0.6, 1.8, 0.6);

    @Test
    void aSphereForbidsByDistanceRatherThanByCorner() {
        AreaInfluence breath = AreaInfluence.forbidding(
                Area.sphere(new BlockVec(0, 64, 0), 8), "dragon-breath");

        assertTrue(breath.evaluate(null, new Vec(0.5, 64.5, 0.5), BOX)
                .blocked(), "the centre is inside");
        assertTrue(breath.evaluate(null, new Vec(0.5, 64.5, 7.5), BOX)
                .blocked(), "a point within the radius is inside");
        // A cuboid of the same extent would have caught this corner.
        assertEquals(InfluenceResult.NONE,
                breath.evaluate(null, new Vec(7.5, 71.5, 7.5), BOX),
                "a corner beyond the radius is outside a sphere");
    }

    @Test
    void aCostingAreaPricesRatherThanCloses() {
        AreaInfluence swamp = AreaInfluence.costing(
                Area.cuboid(new BlockVec(0, 64, 0), new BlockVec(4, 66, 4)),
                6, "swamp");

        InfluenceResult inside = swamp.evaluate(null, new Vec(2.5, 65.5, 2.5), BOX);
        assertEquals(6, inside.costDelta(), 1.0e-9);
        assertTrue(!inside.blocked(), "a priced area stays walkable");
        assertEquals(InfluenceResult.NONE,
                swamp.evaluate(null, new Vec(9.5, 65.5, 9.5), BOX));
    }

    @Test
    void equalAreasCompareEqualSoMobsCanShareARoute() {
        AreaInfluence first = AreaInfluence.forbidding(
                Area.sphere(new BlockVec(1, 2, 3), 5), "zone");
        AreaInfluence same = AreaInfluence.forbidding(
                Area.sphere(new BlockVec(1, 2, 3), 5), "zone");
        AreaInfluence wider = AreaInfluence.forbidding(
                Area.sphere(new BlockVec(1, 2, 3), 6), "zone");

        assertEquals(first, same, "the same region is the same influence");
        assertEquals(first.hashCode(), same.hashCode());
        assertNotEquals(first, wider);
        assertEquals(List.of(first),
                NavigationInfluence.byDeclaredEquality(List.of(same)),
                "declared equality carries through to mesh sharing");
    }

    @Test
    void invalidAreaInfluencesAreRejected() {
        Area area = Area.cuboid(new BlockVec(0, 0, 0), new BlockVec(1, 1, 1));
        assertThrows(IllegalArgumentException.class,
                () -> new AreaInfluence(null, true, 0, "n"));
        assertThrows(IllegalArgumentException.class,
                () -> new AreaInfluence(area, true, 0, null));
        assertThrows(IllegalArgumentException.class,
                () -> new AreaInfluence(area, false, -1, "n"));
        assertThrows(IllegalArgumentException.class,
                () -> new AreaInfluence(area, false, Double.NaN, "n"));
    }

    @Test
    void theSharedMeshIsNotATickableBecauseItsArgumentIsATickId() {
        assertTrue(!Tickable.class.isAssignableFrom(SharedMeshNavigation.class),
                "Tickable passes milliseconds, but the mesh compares its "
                        + "argument against idle-tick policies and uses it to "
                        + "swallow repeated pursuit ticks");
    }
}
