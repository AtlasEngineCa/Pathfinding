package ca.atlasengine.pathfinding.search;

/**
 * The three bounds a discrete ground search runs under.
 *
 * <p>They used to sit adjacent in a parameter list as {@code (64, 0, 1)},
 * where the two doubles could be transposed into a search with a sixteenth of
 * the intended reach. Reach an instance through {@link #ofPathLength(double)}
 * or {@link #builder()}.</p>
 */
public final class GroundSearchLimits {
    private final double maxPathLength;
    private final int reachRange;
    private final double maxVisitedMultiplier;

    private GroundSearchLimits(double maxPathLength, int reachRange,
                               double maxVisitedMultiplier) {
        if (!Double.isFinite(maxPathLength) || maxPathLength <= 0
                || reachRange < 0
                || !Double.isFinite(maxVisitedMultiplier)
                || maxVisitedMultiplier <= 0) {
            throw new IllegalArgumentException("invalid ground search limits");
        }
        this.maxPathLength = maxPathLength;
        this.reachRange = reachRange;
        this.maxVisitedMultiplier = maxVisitedMultiplier;
    }

    /**
     * The common case: a geometric path-length bound, an exact destination,
     * and the unscaled expansion budget.
     */
    public static GroundSearchLimits ofPathLength(double maxPathLength) {
        return new GroundSearchLimits(maxPathLength, 0, 1);
    }

    public static Builder builder() {
        return new Builder();
    }

    /** How far an expanded node may sit from the requested start. */
    public double maxPathLength() {
        return maxPathLength;
    }

    /** How close to the goal counts as reaching it. */
    public int reachRange() {
        return reachRange;
    }

    /** Scales the expansion budget derived from {@link #maxPathLength()}. */
    public double maxVisitedMultiplier() {
        return maxVisitedMultiplier;
    }

    public GroundSearchLimits withReachRange(int value) {
        return new GroundSearchLimits(
                maxPathLength, value, maxVisitedMultiplier);
    }

    public GroundSearchLimits withMaxVisitedMultiplier(double value) {
        return new GroundSearchLimits(maxPathLength, reachRange, value);
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (!(object instanceof GroundSearchLimits other)) return false;
        return Double.compare(maxPathLength, other.maxPathLength) == 0
                && reachRange == other.reachRange
                && Double.compare(maxVisitedMultiplier,
                        other.maxVisitedMultiplier) == 0;
    }

    @Override
    public int hashCode() {
        int result = Double.hashCode(maxPathLength);
        result = 31 * result + Integer.hashCode(reachRange);
        return 31 * result + Double.hashCode(maxVisitedMultiplier);
    }

    @Override
    public String toString() {
        return "GroundSearchLimits[maxPathLength=" + maxPathLength
                + ", reachRange=" + reachRange
                + ", maxVisitedMultiplier=" + maxVisitedMultiplier + "]";
    }

    /** Reach range defaults to zero and the visit multiplier to one. */
    public static final class Builder {
        private double maxPathLength = 64;
        private int reachRange;
        private double maxVisitedMultiplier = 1;

        private Builder() {
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

        public GroundSearchLimits build() {
            return new GroundSearchLimits(
                    maxPathLength, reachRange, maxVisitedMultiplier);
        }
    }
}
