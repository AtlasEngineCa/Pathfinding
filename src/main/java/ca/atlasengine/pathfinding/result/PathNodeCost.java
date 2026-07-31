package ca.atlasengine.pathfinding.result;

import ca.atlasengine.pathfinding.terrain.TerrainType;

/**
 * The terrain classification and node malus a search charged for entering one
 * graph cell. Search-local maluses are not readable from a mob profile, so
 * this is the only view of the cost a route was actually priced at.
 */
public record PathNodeCost(int graphX, int graphY, int graphZ,
                           TerrainType terrain, double malus) {
    public PathNodeCost {
        if (terrain == null) throw new IllegalArgumentException("terrain");
        if (!Double.isFinite(malus)) {
            throw new IllegalArgumentException("malus must be finite");
        }
    }
}
