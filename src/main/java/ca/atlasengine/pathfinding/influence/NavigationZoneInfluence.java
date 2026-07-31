package ca.atlasengine.pathfinding.influence;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;
import static ca.atlasengine.pathfinding.internal.Points.finite;

/**
 * Arbitrary axis-aligned forbidden or costly navigation zone.
 *
 * <p>The canonical constructor takes continuous world coordinates, so a zone
 * built straight from the block coordinates it means to cover stops at the
 * near face of the maximum cell and leaves that whole cell walkable. Use
 * {@link #blocks} to name block cells and the continuous constructor only for
 * sub-block bounds.</p>
 */
public record NavigationZoneInfluence(
        Point minimum,
        Point maximum,
        boolean forbidden,
        double cost,
        String name
) implements NavigationInfluence {
    public NavigationZoneInfluence {
        if (minimum == null || maximum == null || name == null
                || !Double.isFinite(cost) || cost < 0
                || !finite(minimum) || !finite(maximum)
                || minimum.x() > maximum.x()
                || minimum.y() > maximum.y()
                || minimum.z() > maximum.z()) {
            throw new IllegalArgumentException("invalid navigation zone");
        }
        minimum = new Vec(minimum.x(), minimum.y(), minimum.z());
        maximum = new Vec(maximum.x(), maximum.y(), maximum.z());
    }


    /**
     * A zone covering whole block cells, inclusive of both named corners in any
     * order. The cells are expanded to the continuous volume they occupy, so
     * every cell named here is genuinely inside the zone.
     */
    public static NavigationZoneInfluence blocks(
            Point firstBlock, Point lastBlock, boolean forbidden,
            double cost, String name) {
        if (firstBlock == null || lastBlock == null) {
            throw new IllegalArgumentException("invalid navigation zone");
        }
        return new NavigationZoneInfluence(
                BlockCells.minimumCorner(firstBlock, lastBlock),
                BlockCells.maximumCorner(firstBlock, lastBlock),
                forbidden, cost, name);
    }

    @Override
    public InfluenceResult evaluate(Block.Getter blocks, Point point, BoundingBox box) {
        if (!overlaps(point, box)) return InfluenceResult.NONE;
        return forbidden ? InfluenceResult.forbidden("zone:" + name)
                : InfluenceResult.penalty(cost, "zone:" + name);
    }

    /** Returns true as soon as any part of the entity's box enters the zone. */
    private boolean overlaps(Point position, BoundingBox box) {
        return position.x() + box.maxX() >= minimum.x()
                && position.x() + box.minX() <= maximum.x()
                && position.y() + box.maxY() >= minimum.y()
                && position.y() + box.minY() <= maximum.y()
                && position.z() + box.maxZ() >= minimum.z()
                && position.z() + box.minZ() <= maximum.z();
    }
}
