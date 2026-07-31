package ca.atlasengine.pathfinding.influence;

import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.EntityType;

import java.util.UUID;

/**
 * Immutable entity data captured on the instance tick thread for worker use.
 */
public record EntitySnapshot(UUID uuid, EntityType type, Vec position) {
    public EntitySnapshot {
        if (uuid == null || type == null || position == null
                || !Double.isFinite(position.x())
                || !Double.isFinite(position.y())
                || !Double.isFinite(position.z())) {
            throw new IllegalArgumentException("snapshot fields");
        }
    }

    public static EntitySnapshot capture(Entity entity) {
        if (entity == null) throw new IllegalArgumentException("entity");
        Point position = entity.getPosition();
        return new EntitySnapshot(entity.getUuid(), entity.getEntityType(),
                new Vec(position.x(), position.y(), position.z()));
    }
}
