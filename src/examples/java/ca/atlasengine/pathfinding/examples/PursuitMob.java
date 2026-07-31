package ca.atlasengine.pathfinding.examples;

import ca.atlasengine.pathfinding.adaptive.SharedMeshPursuit;
import ca.atlasengine.pathfinding.adaptive.SharedMeshSource;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;

/**
 * A crowd mob that pursues one entity through the shared mesh.
 *
 * <p>{@link SharedMeshPursuit} owns the controller and ticks it, so
 * this entity ticks the pursuit instead. The world revision is constant here
 * because the plaza the crowd runs on never changes; a game that edits terrain
 * must increment it whenever retained collision topology goes stale.</p>
 */
public final class PursuitMob extends EntityCreature implements AutoCloseable {
    private static final long WORLD_REVISION = 1;

    private SharedMeshPursuit pursuit;
    private long tick;

    public PursuitMob(EntityType type) {
        super(type);
    }

    public PursuitMob pursue(SharedMeshPursuit navigation) {
        this.pursuit = navigation;
        return this;
    }

    public SharedMeshSource source() {
        return pursuit.source();
    }

    @Override
    public void update(long time) {
        if (pursuit != null && !pursuit.closed()) {
            pursuit.tick(WORLD_REVISION, ++tick);
        }
        super.update(time);
    }

    @Override
    public void close() {
        if (pursuit != null) pursuit.close();
        remove();
    }
}
