package ca.atlasengine.pathfinding.event;

import ca.atlasengine.pathfinding.EntityNavigationController;

/** A navigation was cancelled, including by close(). */
public final class NavigationCancelledEvent extends EntityNavigationEvent {
    public NavigationCancelledEvent(EntityNavigationController controller) {
        super(controller);
    }
}
