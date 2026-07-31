package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.internal.movement.RouteGeometry;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class LiveTraversalSnapshotTest {
    @Test
    void wideEntityDetectsWaterAwayFromItsCenterColumn() {
        TestWorld world = new TestWorld().set(1, 1, 0, Block.WATER);
        BoundingBox wide = new BoundingBox(2.0, 1.8, 0.8);

        assertTrue(RouteGeometry.volumeTouchesWater(
                world, new Vec(0.5, 1, 0.5), wide));
    }

    @Test
    void waterloggedCellCountsAsFluidContact() {
        TestWorld world = new TestWorld().set(
                0, 1, 0, Block.OAK_SLAB
                        .withProperty("waterlogged", "true"));

        assertTrue(RouteGeometry.volumeTouchesWater(
                world, new Vec(0.5, 1, 0.5),
                new BoundingBox(0.6, 1.8, 0.6)));
        assertFalse(RouteGeometry.volumeTouchesWater(
                new TestWorld(), new Vec(0.5, 1, 0.5),
                new BoundingBox(0.6, 1.8, 0.6)));
    }
}
