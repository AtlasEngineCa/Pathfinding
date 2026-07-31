package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.terrain.TerrainType;

/**
 * One graph cell. Evaluator state and A* bookkeeping share the object to avoid
 * parallel maps and per-relaxation allocations.
 */
final class SearchNode {
    final int x;
    final int y;
    final int z;
    TerrainType type = TerrainType.BLOCKED;
    TerrainType classifiedType;
    boolean typeComputed;
    double malus = Double.NEGATIVE_INFINITY;
    /**
     * The malus charged when this node was last relaxed. Evaluators keep
     * mutating {@link #malus} after a node closes, so the accepted cost has
     * to be captured at relaxation time.
     */
    double chargedMalus;
    boolean hardBlocked;
    boolean settled;
    float travelCost;
    float estimate;
    float rank;
    float routeLength;
    SearchNode previous;
    int frontierSlot = -1;
    double floor;
    boolean floorComputed;

    SearchNode(int x, int y, int z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    boolean inOpenSet() {
        return frontierSlot >= 0;
    }

}
