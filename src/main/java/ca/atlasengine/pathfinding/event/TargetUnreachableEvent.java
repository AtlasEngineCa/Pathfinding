package ca.atlasengine.pathfinding.event;

import ca.atlasengine.pathfinding.EntityNavigationController;

/** No further search reaches the current target. */
public final class TargetUnreachableEvent extends EntityNavigationEvent {
    public TargetUnreachableEvent(EntityNavigationController controller) {
        super(controller);
    }
}
