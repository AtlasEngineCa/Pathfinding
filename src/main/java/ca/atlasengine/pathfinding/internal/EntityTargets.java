package ca.atlasengine.pathfinding.internal;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;

/** Target projections used when navigating toward another entity. */
public final class EntityTargets {
    private static final double CONTACT_EPSILON = 1.0e-4;

    private EntityTargets() {
    }

    /**
     * Anchors a grounded entity to the block supporting its footprint. This
     * keeps a sneaking target over an edge associated with the surface it is
     * standing on rather than the empty column beneath its raw position.
     */
    public static Point supportedPosition(Entity entity) {
        if (entity == null) throw new IllegalArgumentException("entity");
        Point position = entity.getPosition();
        Instance instance = entity.getInstance();
        if (instance == null || !entity.isOnGround()) return position;

        BoundingBox box = entity.getBoundingBox();
        double feet = position.y() + box.minY();
        int supportY = (int) Math.floor(feet - CONTACT_EPSILON);
        int minX = cell(position.x() + box.minX() + CONTACT_EPSILON);
        int maxX = cell(position.x() + box.maxX() - CONTACT_EPSILON);
        int minZ = cell(position.z() + box.minZ() + CONTACT_EPSILON);
        int maxZ = cell(position.z() + box.maxZ() - CONTACT_EPSILON);

        int selectedX = position.blockX();
        int selectedZ = position.blockZ();
        double selectedDistance = Double.POSITIVE_INFINITY;
        for (int x = minX; x <= maxX; x++) {
            for (int z = minZ; z <= maxZ; z++) {
                Block support = instance.getBlock(
                        x, supportY, z, Block.Getter.Condition.TYPE);
                double top = supportY
                        + support.collisionShape().relativeEnd().y();
                if (top <= supportY || Math.abs(top - feet) > 0.05) continue;
                double dx = x + 0.5 - position.x();
                double dz = z + 0.5 - position.z();
                double distance = dx * dx + dz * dz;
                if (distance < selectedDistance) {
                    selectedDistance = distance;
                    selectedX = x;
                    selectedZ = z;
                }
            }
        }
        return Double.isFinite(selectedDistance)
                ? new Vec(selectedX + 0.5, position.y(), selectedZ + 0.5)
                : position;
    }

    private static int cell(double coordinate) {
        return (int) Math.floor(coordinate);
    }
}
