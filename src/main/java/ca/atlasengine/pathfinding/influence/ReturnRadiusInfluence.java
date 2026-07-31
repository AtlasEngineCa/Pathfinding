package ca.atlasengine.pathfinding.influence;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;
import static ca.atlasengine.pathfinding.internal.Points.finite;

/**
 * Keeps navigation within a home radius while still allowing an entity that
 * starts outside that radius to make progress back toward its home.
 */
public record ReturnRadiusInfluence(
        Point home,
        Point requestStart,
        double radius
) implements NavigationInfluence {
    public ReturnRadiusInfluence {
        if (home == null || requestStart == null
                || !finite(home) || !finite(requestStart)
                || !Double.isFinite(radius) || radius < 0) {
            throw new IllegalArgumentException("invalid return radius");
        }
        home = new Vec(home.x(), home.y(), home.z());
        requestStart = new Vec(
                requestStart.x(), requestStart.y(), requestStart.z());
    }


    @Override
    public InfluenceResult evaluate(
            Block.Getter blocks, Point point, BoundingBox box) {
        double candidateDistance = squaredDistance(home, point);
        double startDistance = squaredDistance(home, requestStart);
        if (candidateDistance > radius * radius
                && candidateDistance >= startDistance) {
            return InfluenceResult.forbidden("outside_return_radius");
        }
        return InfluenceResult.NONE;
    }

    private static double squaredDistance(Point first, Point second) {
        double dx = first.blockX() - second.blockX();
        double dy = first.blockY() - second.blockY();
        double dz = first.blockZ() - second.blockZ();
        return dx * dx + dy * dy + dz * dz;
    }
}
