package ca.atlasengine.pathfinding.internal.adaptive;

import ca.atlasengine.pathfinding.search.EntityPathfinder;import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;

/**
 * The complete projection of a destination that a search can observe.
 *
 * <p>Two destinations with the same projection are the same search input, so
 * one A* answer is exact for both. The projection is exhaustive rather than
 * approximate:</p>
 *
 * <ul>
 *   <li>{@code PathSearch} ranks and reaches destinations through
 *       {@code blockX/blockY/blockZ} only;</li>
 *   <li>{@code EntityPathfinder.normalizeGroundTarget} reads only those same
 *       block coordinates before resolving a surface cell;</li>
 *   <li>the amphibious entry point additionally rounds the destination to
 *       {@code floor(y + 0.5)}, which is the only sub-block quantity any
 *       search derives from a destination.</li>
 * </ul>
 *
 * <p>A source position has no such projection: the ground evaluator rounds
 * {@code startPosition.y() + 0.5}, scans bounding-box corners at
 * {@code floor(startPosition.x() + box.relativeStart().x())}, and ray-marches
 * partial-collision neighbours from the continuous start. Sources are
 * therefore keyed exactly and never merged.</p>
 */
public record TargetCell(int x, int y, int z, int roundedY) {
    public static TargetCell of(Point point) {
        if (point == null) throw new IllegalArgumentException("point");
        return new TargetCell(point.blockX(), point.blockY(), point.blockZ(),
                (int) Math.floor(point.y() + 0.5));
    }

    /** A destination of this class, used to store one plan per class. */
    public Point representative() {
        return new Vec(x, roundedY == y ? y : y + 0.5, z);
    }
}
