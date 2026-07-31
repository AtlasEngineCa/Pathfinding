package ca.atlasengine.pathfinding;

public enum NavigationState {
    IDLE,
    COMPUTING,
    FOLLOWING,
    COMPLETED,
    PARTIAL,
    STUCK,
    FAILED,
    CANCELLED
}
