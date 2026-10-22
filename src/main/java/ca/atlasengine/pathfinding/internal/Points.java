package ca.atlasengine.pathfinding.internal;

import net.minestom.server.coordinate.Point;

import java.util.List;

/**
 * Coordinate validation shared by every entry point that accepts a caller
 * supplied position.
 */
public final class Points {
    private Points() {
    }

    /**
     * Whether every component of {@code point} is finite.
     *
     * <p>{@link Point#blockX()} floors NaN to 0, so an unvalidated corrupt
     * coordinate collapses onto the origin cell and the search reports FOUND
     * immediately.</p>
     */
    public static boolean finite(Point point) {
        return Double.isFinite(point.x())
                && Double.isFinite(point.y())
                && Double.isFinite(point.z());
    }

    /** Whether {@code points} is non-empty and every element is finite. */
    public static boolean allFinite(List<? extends Point> points) {
        if (points == null || points.isEmpty()) return false;
        for (Point point : points) {
            if (point == null || !finite(point)) return false;
        }
        return true;
    }
}
