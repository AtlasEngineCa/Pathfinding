package ca.atlasengine.pathfinding.influence;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block;

import java.util.Comparator;
import java.util.List;
import java.util.Objects;

/**
 * Radial fear field around immutable entity snapshots.
 */
public final class EntityFearInfluence implements NavigationInfluence {
    private static final Comparator<EntitySnapshot> CANONICAL_ORDER =
            Comparator.comparing(EntitySnapshot::uuid)
                    .thenComparing(threat -> threat.type().key().asString())
                    .thenComparingDouble(threat -> threat.position().x())
                    .thenComparingDouble(threat -> threat.position().y())
                    .thenComparingDouble(threat -> threat.position().z());

    private final List<EntitySnapshot> threats;
    private final double radius;
    private final double forbiddenRadius;
    private final double maximumPenalty;

    public EntityFearInfluence(List<EntitySnapshot> threats, double radius,
                               double forbiddenRadius, double maximumPenalty) {
        if (threats == null || threats.stream().anyMatch(Objects::isNull)
                || !Double.isFinite(radius) || radius <= 0
                || !Double.isFinite(forbiddenRadius) || forbiddenRadius < 0
                || forbiddenRadius > radius
                || !Double.isFinite(maximumPenalty) || maximumPenalty < 0) {
            throw new IllegalArgumentException("invalid fear influence");
        }
        // evaluate charges every threat, never the list, so the order a
        // caller collected them in is not part of this influence.
        this.threats = threats.stream().sorted(CANONICAL_ORDER).toList();
        this.radius = radius;
        this.forbiddenRadius = forbiddenRadius;
        this.maximumPenalty = maximumPenalty;
    }

    @Override
    public InfluenceResult evaluate(Block.Getter blocks, Point point, BoundingBox boundingBox) {
        double total = 0;
        for (EntitySnapshot threat : threats) {
            double distance = point.distance(threat.position());
            if (distance <= forbiddenRadius) {
                return InfluenceResult.forbidden("feared_entity:" + threat.type().key());
            }
            if (distance < radius) {
                double fraction = 1.0 - distance / radius;
                total += maximumPenalty * fraction * fraction;
            }
        }
        return total == 0 ? InfluenceResult.NONE
                : InfluenceResult.penalty(total, "entity_fear");
    }

    /**
     * Exact equality over the captured threat list. Two fear fields are only
     * interchangeable when every threat identity and snapshot position agrees,
     * so a threat that moved at all separates them again. The list is held in
     * a canonical order, so the order two callers collected one threat set in
     * never separates them, while a threat listed twice, which is charged
     * twice, still does.
     */
    @Override
    public boolean equals(Object object) {
        return object instanceof EntityFearInfluence other
                && Double.compare(radius, other.radius) == 0
                && Double.compare(forbiddenRadius, other.forbiddenRadius) == 0
                && Double.compare(maximumPenalty, other.maximumPenalty) == 0
                && threats.equals(other.threats);
    }

    @Override
    public int hashCode() {
        return Objects.hash(threats, radius, forbiddenRadius, maximumPenalty);
    }
}
