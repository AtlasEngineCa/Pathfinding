package ca.atlasengine.pathfinding.terrain;

/**
 * Block families carrying an {@code open} state a mob could toggle.
 *
 * <p>Only {@link #DOOR} is manipulated by baseline mobs. The other two exist
 * so an integration can opt into behavior Minecraft itself never gives a
 * mob.</p>
 */
public enum OpenableBlockFamily {
    DOOR,
    TRAPDOOR,
    FENCE_GATE
}
