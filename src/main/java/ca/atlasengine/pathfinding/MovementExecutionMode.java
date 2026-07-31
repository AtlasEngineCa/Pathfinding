package ca.atlasengine.pathfinding;

/**
 * Selects how a navigation controller turns waypoints into entity movement.
 */
public enum MovementExecutionMode {
    /**
     * Applies collision-resolved displacement immediately. Useful for
     * deterministic compatibility and isolated controller tests.
     */
    DIRECT_COLLISION,

    /**
     * Supplies velocity and lets Minestom's normal entity tick apply
     * collision, gravity, drag, and on-ground state.
     */
    PHYSICS_VELOCITY
}
