package ca.atlasengine.pathfinding.influence;

import ca.atlasengine.pathfinding.search.SearchControl;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block;

import java.util.Objects;
import java.util.Set;

/**
 * Avoids the given block types within a configurable block radius.
 *
 * <p>Blocks are compared by identity rather than by name, so block state is
 * ignored and a modded block never collides with the baseline block of the
 * same short name.</p>
 */
public final class BlockAvoidanceInfluence implements NavigationInfluence {
    public static final int MAX_RADIUS = 8;
    private static final int CANCELLATION_CHECK_INTERVAL = 64;
    private final Set<Block> avoided;
    private final int radius;
    private final boolean forbidden;
    private final double penalty;

    public BlockAvoidanceInfluence(Set<Block> avoided, int radius,
                                   boolean forbidden, double penalty) {
        if (avoided == null || avoided.isEmpty()
                || avoided.stream().anyMatch(Objects::isNull)
                || radius < 0 || radius > MAX_RADIUS
                || !Double.isFinite(penalty) || penalty < 0) {
            throw new IllegalArgumentException("invalid block avoidance");
        }
        this.avoided = Set.copyOf(avoided);
        this.radius = radius;
        this.forbidden = forbidden;
        this.penalty = penalty;
    }

    @Override
    public InfluenceResult evaluate(Block.Getter blocks, Point point, BoundingBox boundingBox) {
        return evaluate(blocks, point, boundingBox, SearchControl.NONE);
    }

    @Override
    public InfluenceResult evaluate(
            Block.Getter blocks, Point point, BoundingBox boundingBox,
            SearchControl control) {
        int reads = 0;
        for (int x = point.blockX() - radius; x <= point.blockX() + radius; x++) {
            for (int y = point.blockY() - radius; y <= point.blockY() + radius; y++) {
                for (int z = point.blockZ() - radius; z <= point.blockZ() + radius; z++) {
                    if ((reads++ & (CANCELLATION_CHECK_INTERVAL - 1)) == 0
                            && (control.cancelled() || control.timedOut())) {
                        return InfluenceResult.forbidden(
                                "avoidance_scan_interrupted");
                    }
                    Block found = blocks.getBlock(
                            x, y, z, Block.Getter.Condition.TYPE);
                    if (matches(found)) {
                        String name = found.key().asString();
                        return forbidden
                                ? InfluenceResult.forbidden("avoided_block:" + name)
                                : InfluenceResult.penalty(penalty, "avoided_block:" + name);
                    }
                }
            }
        }
        return InfluenceResult.NONE;
    }

    @Override
    public boolean equals(Object object) {
        return object instanceof BlockAvoidanceInfluence other
                && radius == other.radius
                && forbidden == other.forbidden
                && Double.compare(penalty, other.penalty) == 0
                && avoided.equals(other.avoided);
    }

    @Override
    public int hashCode() {
        return Objects.hash(avoided, radius, forbidden, penalty);
    }

    /** Identity comparison, so block state never changes the answer. */
    private boolean matches(Block block) {
        for (Block candidate : avoided) {
            if (block.compare(candidate)) return true;
        }
        return false;
    }
}
