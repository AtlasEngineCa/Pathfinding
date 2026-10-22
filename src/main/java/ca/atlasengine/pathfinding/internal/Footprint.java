package ca.atlasengine.pathfinding.internal;

import net.minestom.server.collision.BoundingBox;

/**
 * How many block cells an entity box spans when anchored at a graph cell's
 * minimum corner. Every evaluator, classifier, and waypoint projection sizes
 * its scans by this one rule.
 */
public final class Footprint {
    private Footprint() {
    }

    /** Cells covered along one axis by a box of this extent. */
    public static int cells(double extent) {
        return Math.max(1, (int) Math.floor(extent + 1));
    }

    public static int width(BoundingBox box) {
        return cells(box.width());
    }

    public static int height(BoundingBox box) {
        return cells(box.height());
    }

    public static int depth(BoundingBox box) {
        return cells(box.depth());
    }

    /** Mean of the three extents, which separates small flyers from large. */
    public static double meanExtent(BoundingBox box) {
        return (box.width() + box.height() + box.depth()) / 3;
    }
}
