package ca.atlasengine.pathfinding.influence;

import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;

/**
 * The continuous volume a range of block cells occupies. A cell spans one whole
 * unit, so the volume enclosing cells {@code first} through {@code last} ends
 * one past the maximum cell on every axis. Every navigation type that accepts
 * block cells expands them here, so the rule is stated once.
 */
final class BlockCells {
    private BlockCells() {
    }

    static Vec minimumCorner(Point first, Point second) {
        return new Vec(
                Math.min(first.blockX(), second.blockX()),
                Math.min(first.blockY(), second.blockY()),
                Math.min(first.blockZ(), second.blockZ()));
    }

    static Vec maximumCorner(Point first, Point second) {
        return new Vec(
                Math.max(first.blockX(), second.blockX()) + 1.0,
                Math.max(first.blockY(), second.blockY()) + 1.0,
                Math.max(first.blockZ(), second.blockZ()) + 1.0);
    }
}
