package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.profile.GroundCapabilities;

/**
 * Search limits, movement capabilities, and amphibious settings for one
 * ground search.
 */
public record GroundSearchSpec(double maxPathLength, int reachRange, int maxVisited,
                               GroundCapabilities capabilities, boolean amphibious,
                               boolean prefersJumpToPreferredBlocks,
                               boolean prefersShallowWater, int seaLevel) {
    public static final int OVERWORLD_SEA_LEVEL = 63;

    public static GroundSearchSpec walking(double maxPathLength, int reachRange,
                                           int maxVisited,
                                           GroundCapabilities capabilities) {
        return new GroundSearchSpec(maxPathLength, reachRange, maxVisited,
                capabilities, false, false, false, OVERWORLD_SEA_LEVEL);
    }

    public static GroundSearchSpec amphibious(double maxPathLength, int reachRange,
                                              int maxVisited,
                                              GroundCapabilities capabilities,
                                              boolean prefersJumpToPreferredBlocks,
                                              boolean prefersShallowWater,
                                              int seaLevel) {
        return new GroundSearchSpec(maxPathLength, reachRange, maxVisited,
                capabilities, true, prefersJumpToPreferredBlocks,
                prefersShallowWater, seaLevel);
    }
}
