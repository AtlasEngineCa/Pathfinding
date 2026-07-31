package ca.atlasengine.pathfinding.search;

import ca.atlasengine.pathfinding.profile.NavigationModifiers;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.internal.search.GroundSearchSpec;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block;

import java.util.List;
import java.util.Objects;
import static ca.atlasengine.pathfinding.internal.Points.finite;

/**
 * Immutable input for one entity-aware path search.
 *
 * <p>Use {@link #builder(Block.Getter, Point, Point, BoundingBox,
 * NavigationProfile)} so search limits are named at the call site.</p>
 */
public record NavigationRequest(
        Block.Getter blocks,
        Point start,
        Point target,
        BoundingBox boundingBox,
        NavigationProfile profile,
        double maxPathLength,
        int reachRange,
        double maxVisitedMultiplier,
        List<NavigationInfluence> influences,
        int seaLevel,
        EntityTraversalState entityState,
        double nodeSearchRange
) {
    /** Hard ceiling on per-node footprint work, well above any baseline mob. */
    public static final double MAX_BOUNDING_BOX_EXTENT = 64;
    /** Bounds the cubic block scan performed when classifying one graph cell. */
    public static final int MAX_FOOTPRINT_CELLS = 4096;

    public NavigationRequest {
        if (blocks == null || start == null || target == null || boundingBox == null
                || profile == null || !Double.isFinite(maxPathLength) || maxPathLength <= 0
                || reachRange < 0 || !Double.isFinite(maxVisitedMultiplier)
                || maxVisitedMultiplier <= 0 || influences == null
                || influences.stream().anyMatch(Objects::isNull)
                || entityState == null || !Double.isFinite(nodeSearchRange)
                || nodeSearchRange <= 0
                || !finite(start) || !finite(target)
                || !valid(boundingBox)) {
            throw new IllegalArgumentException("invalid navigation request");
        }
        influences = List.copyOf(influences);
    }


    private static boolean valid(BoundingBox box) {
        return finite(box.relativeStart()) && finite(box.relativeEnd())
                && box.width() > 0 && box.height() > 0 && box.depth() > 0
                && box.width() <= MAX_BOUNDING_BOX_EXTENT
                && box.height() <= MAX_BOUNDING_BOX_EXTENT
                && box.depth() <= MAX_BOUNDING_BOX_EXTENT
                && footprintCells(box) <= MAX_FOOTPRINT_CELLS;
    }

    private static long footprintCells(BoundingBox box) {
        long width = Math.max(1, (long) Math.floor(box.width() + 1));
        long height = Math.max(1, (long) Math.floor(box.height() + 1));
        long depth = Math.max(1, (long) Math.floor(box.depth() + 1));
        return width * height * depth;
    }

    /**
     * Starts a request with practical defaults: a 64-block path budget,
     * exact target reach, normal visited-node budget, sea level 63, grounded
     * state, no dynamic influences, and a node search range derived from the
     * path budget.
     */
    public static Builder builder(
            Block.Getter blocks, Point start, Point target,
            BoundingBox boundingBox, NavigationProfile profile) {
        return new Builder(blocks, start, target, boundingBox, profile);
    }

    public NavigationRequest withModifiers(NavigationModifiers modifiers) {
        if (modifiers == null) throw new IllegalArgumentException("modifiers");
        return new NavigationRequest(
                blocks, start, target, boundingBox, modifiers.applyTo(profile),
                maxPathLength, reachRange, maxVisitedMultiplier, influences,
                seaLevel, entityState, nodeSearchRange);
    }

    public static final class Builder {
        private final Block.Getter blocks;
        private final Point start;
        private final Point target;
        private final BoundingBox boundingBox;
        private final NavigationProfile profile;
        private double maxPathLength = 64;
        private int reachRange;
        private double maxVisitedMultiplier = 1;
        private List<NavigationInfluence> influences = List.of();
        private int seaLevel = GroundSearchSpec.OVERWORLD_SEA_LEVEL;
        private EntityTraversalState entityState = EntityTraversalState.GROUNDED;
        private double nodeSearchRange;
        private boolean explicitNodeSearchRange;

        private Builder(
                Block.Getter blocks, Point start, Point target,
                BoundingBox boundingBox, NavigationProfile profile) {
            this.blocks = blocks;
            this.start = start;
            this.target = target;
            this.boundingBox = boundingBox;
            this.profile = profile;
        }

        public Builder maxPathLength(double value) {
            maxPathLength = value;
            return this;
        }

        public Builder reachRange(int value) {
            reachRange = value;
            return this;
        }

        public Builder maxVisitedMultiplier(double value) {
            maxVisitedMultiplier = value;
            return this;
        }

        public Builder influences(List<NavigationInfluence> value) {
            influences = value;
            return this;
        }

        public Builder seaLevel(int value) {
            seaLevel = value;
            return this;
        }

        public Builder entityState(EntityTraversalState value) {
            entityState = value;
            return this;
        }

        public Builder nodeSearchRange(double value) {
            nodeSearchRange = value;
            explicitNodeSearchRange = true;
            return this;
        }

        public NavigationRequest build() {
            double effectiveNodeSearchRange = explicitNodeSearchRange
                    ? nodeSearchRange : Math.max(16, maxPathLength);
            return new NavigationRequest(
                    blocks, start, target, boundingBox, profile,
                    maxPathLength, reachRange, maxVisitedMultiplier,
                    influences, seaLevel, entityState,
                    effectiveNodeSearchRange);
        }
    }
}
