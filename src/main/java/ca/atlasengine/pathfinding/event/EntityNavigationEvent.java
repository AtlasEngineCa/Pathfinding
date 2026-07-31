package ca.atlasengine.pathfinding.event;

import ca.atlasengine.pathfinding.EntityNavigationController;
import net.minestom.server.entity.Entity;
import net.minestom.server.event.trait.EntityEvent;

/**
 * A navigation event about one entity, filterable with
 * {@link net.minestom.server.event.EventFilter#ENTITY}.
 */
public abstract class EntityNavigationEvent
        implements NavigationEvent, EntityEvent {
    private final EntityNavigationController controller;

    protected EntityNavigationEvent(EntityNavigationController controller) {
        if (controller == null) throw new IllegalArgumentException("controller");
        this.controller = controller;
    }

    /** The controller that announced this, already in its new state. */
    public EntityNavigationController controller() {
        return controller;
    }

    @Override
    public Entity getEntity() {
        return controller.entity();
    }
}
