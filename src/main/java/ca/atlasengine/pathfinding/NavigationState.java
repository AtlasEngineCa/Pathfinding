package ca.atlasengine.pathfinding;

public enum NavigationState {
    IDLE,
    COMPUTING,
    FOLLOWING,
    COMPLETED,
    PARTIAL,
    STUCK,
    FAILED,
    CANCELLED;

    /**
     * Whether this state ends a navigation. A controller in a terminal state
     * stays there until it is given a new target or plan.
     */
    public boolean terminal() {
        return switch (this) {
            case COMPLETED, STUCK, FAILED, CANCELLED -> true;
            case IDLE, COMPUTING, FOLLOWING, PARTIAL -> false;
        };
    }
}
