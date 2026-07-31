package ca.atlasengine.pathfinding.internal.search;

import net.minestom.server.coordinate.Point;

record CellKey(int x, int y, int z) {
    static CellKey of(Point point) {
        return new CellKey(point.blockX(), point.blockY(), point.blockZ());
    }
}
