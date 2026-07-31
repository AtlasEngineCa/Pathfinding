package ca.atlasengine.pathfinding.influence;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Area;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block;

/**
 * A forbidden or costly region described by a Minestom {@link Area}.
 *
 * <p>{@link NavigationZoneInfluence} covers an axis-aligned box in continuous
 * coordinates, which is what a zone thinner than a block needs. This covers
 * everything else Minestom can describe, including
 * {@link Area#sphere(net.minestom.server.coordinate.BlockVec, int) spheres}
 * and {@link Area#line lines}, which a box cannot express at all:</p>
 *
 * <pre>{@code
 * new AreaInfluence(Area.sphere(new BlockVec(0, 64, 0), 12),
 *         true, 0, "dragon-breath");
 * }</pre>
 *
 * <p>An {@code Area} is block-granular, so membership is decided by the block
 * cell a candidate stands in. Every Minestom area is a value, so two
 * influences describing the same region compare equal and mobs carrying them
 * may share a computed route.</p>
 */
public record AreaInfluence(
        Area area,
        boolean forbidden,
        double cost,
        String name
) implements NavigationInfluence {
    public AreaInfluence {
        if (area == null || name == null
                || !Double.isFinite(cost) || cost < 0) {
            throw new IllegalArgumentException("invalid area influence");
        }
    }

    /** Forbids the area outright. */
    public static AreaInfluence forbidding(Area area, String name) {
        return new AreaInfluence(area, true, 0, name);
    }

    /** Prices the area rather than closing it. */
    public static AreaInfluence costing(Area area, double cost, String name) {
        return new AreaInfluence(area, false, cost, name);
    }

    @Override
    public InfluenceResult evaluate(
            Block.Getter blocks, Point point, BoundingBox boundingBox) {
        if (!area.contains(point)) return InfluenceResult.NONE;
        return forbidden
                ? InfluenceResult.forbidden(name)
                : InfluenceResult.penalty(cost, name);
    }
}
