package ca.atlasengine.pathfinding.event;

import ca.atlasengine.pathfinding.EntityNavigationController;

/** The follower could not walk the route it was given. */
public final class NavigationStuckEvent extends EntityNavigationEvent {
    public NavigationStuckEvent(EntityNavigationController controller) {
        super(controller);
    }
}
