package ca.atlasengine.pathfinding.influence;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import static ca.atlasengine.pathfinding.internal.Points.finite;

/**
 * An immutable axis-aligned area in which a complete entity box may move.
 *
 * <p>The canonical constructor takes continuous world coordinates, so an area
 * built straight from the block coordinates it means to allow stops at the near
 * face of the maximum cell and shuts a mob out of that whole cell. Use {@link
 * #blocks} to name block cells and the continuous constructor only for
 * sub-block bounds.</p>
 */
public record NavigationArea(Point minimum, Point maximum, String name) {
    public NavigationArea {
        if (minimum == null || maximum == null || name == null || name.isBlank()
                || !finite(minimum) || !finite(maximum)
                || minimum.x() > maximum.x()
                || minimum.y() > maximum.y()
                || minimum.z() > maximum.z()) {
            throw new IllegalArgumentException("invalid navigation area");
        }
        minimum = new Vec(minimum.x(), minimum.y(), minimum.z());
        maximum = new Vec(maximum.x(), maximum.y(), maximum.z());
    }


    /**
     * An area covering whole block cells, inclusive of both named corners in
     * any order. The cells are expanded to the continuous volume they occupy,
     * so a mob may stand in every cell named here.
     */
    public static NavigationArea blocks(
            Point firstBlock, Point lastBlock, String name) {
        if (firstBlock == null || lastBlock == null) {
            throw new IllegalArgumentException("invalid navigation area");
        }
        return new NavigationArea(
                BlockCells.minimumCorner(firstBlock, lastBlock),
                BlockCells.maximumCorner(firstBlock, lastBlock), name);
    }

    /** Returns true only when the entity's complete box lies inside. */
    public boolean contains(Point position, BoundingBox box) {
        if (position == null || box == null) return false;
        return position.x() + box.minX() >= minimum.x()
                && position.x() + box.maxX() <= maximum.x()
                && position.y() + box.minY() >= minimum.y()
                && position.y() + box.maxY() <= maximum.y()
                && position.z() + box.minZ() >= minimum.z()
                && position.z() + box.maxZ() <= maximum.z();
    }
}
