package ca.atlasengine.pathfinding.terrain;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;

/**
 * The block cells a follower reaches at one waypoint: every cell the entity
 * box covers, over the height the follower opens through.
 *
 * <p>A waypoint is the origin to which the box's relative bounds are applied,
 * not necessarily its centre: Minestom permits offset boxes. A ladder-anchored
 * waypoint can therefore lie outside every reached cell. The relative box is
 * the only definition under which the planner's view and follower's writes
 * cover the same blocks.</p>
 */
public record ReachedCells(int minX, int minY, int minZ,
                           int maxX, int maxY, int maxZ) {
    public static ReachedCells at(Point waypoint, BoundingBox box) {
        if (waypoint == null) throw new IllegalArgumentException("waypoint");
        if (box == null) throw new IllegalArgumentException("box");
        return new ReachedCells(
                cell(waypoint.x() + box.minX()),
                cell(waypoint.y() + box.minY()),
                cell(waypoint.z() + box.minZ()),
                cell(waypoint.x() + box.maxX()),
                cell(waypoint.y() + box.maxY()),
                cell(waypoint.z() + box.maxZ()));
    }

    public boolean contains(int x, int y, int z) {
        return x >= minX && x <= maxX && y >= minY && y <= maxY
                && z >= minZ && z <= maxZ;
    }

    private static int cell(double coordinate) {
        return (int) Math.floor(coordinate);
    }
}
