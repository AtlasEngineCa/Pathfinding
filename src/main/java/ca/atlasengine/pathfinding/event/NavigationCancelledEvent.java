package ca.atlasengine.pathfinding.event;

import ca.atlasengine.pathfinding.EntityNavigationController;

/**
 * A navigation in progress was cancelled, including by closing its controller
 * or removing its entity. A navigation that had already finished is not.
 */
public final class NavigationCancelledEvent extends EntityNavigationEvent {
    public NavigationCancelledEvent(EntityNavigationController controller) {
        super(controller);
    }
}
