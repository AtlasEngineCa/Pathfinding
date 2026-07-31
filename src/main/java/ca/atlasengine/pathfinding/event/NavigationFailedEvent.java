package ca.atlasengine.pathfinding.event;

import ca.atlasengine.pathfinding.EntityNavigationController;

/** A navigation ended in failure. */
public final class NavigationFailedEvent extends EntityNavigationEvent {
    public NavigationFailedEvent(EntityNavigationController controller) {
        super(controller);
    }
}
