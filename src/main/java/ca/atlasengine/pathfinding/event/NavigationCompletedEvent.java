package ca.atlasengine.pathfinding.event;

import ca.atlasengine.pathfinding.EntityNavigationController;

/** The entity reached its destination. */
public final class NavigationCompletedEvent extends EntityNavigationEvent {
    public NavigationCompletedEvent(EntityNavigationController controller) {
        super(controller);
    }
}
