package ca.atlasengine.pathfinding.profile;

/**
 * Opt-in platform-to-platform jumping limits.
 *
 * <p>The horizontal distance is measured between graph-cell anchors. A value
 * of {@code 2} can cross one unsupported cell. Jump edges are generated only
 * across open gaps; ordinary carpets, slabs, stairs, and walkable ground keep
 * using normal walk/step edges.</p>
 *
 * <p>All four limits are distances, so this is not a record: a
 * positional constructor would let any permutation of them compile into a
 * different mob. Reach an instance through {@link #acrossGaps(int)} or
 * {@link #builder()}.</p>
 */
public final class PlatformJumpCapabilities {
    public static final PlatformJumpCapabilities DISABLED =
            new PlatformJumpCapabilities(0, 0, 0, 0);

    private final double maxHorizontalDistance;
    private final double maxRise;
    private final double maxDrop;
    private final double apexClearance;

    private PlatformJumpCapabilities(double maxHorizontalDistance,
                                     double maxRise, double maxDrop,
                                     double apexClearance) {
        if (!Double.isFinite(maxHorizontalDistance)
                || !Double.isFinite(maxRise)
                || !Double.isFinite(maxDrop)
                || !Double.isFinite(apexClearance)
                || maxHorizontalDistance < 0
                || maxHorizontalDistance > 32
                || maxRise < 0 || maxRise > 16
                || maxDrop < 0 || maxDrop > 16
                || apexClearance < 0 || apexClearance > 16) {
            throw new IllegalArgumentException(
                    "invalid platform jump capabilities");
        }
        this.maxHorizontalDistance = maxHorizontalDistance;
        this.maxRise = maxRise;
        this.maxDrop = maxDrop;
        this.apexClearance = apexClearance;
    }

    /**
     * Limits sized by the widest run of unsupported cells to cross on the
     * level, leaving rise and drop at zero.
     *
     * <p>The stored distance is measured between graph-cell anchors, so a
     * footprint wider or deeper than one cell needs the cells it straddles
     * added on top of {@code gapCells}.</p>
     */
    public static PlatformJumpCapabilities acrossGaps(int gapCells) {
        if (gapCells < 0 || gapCells > 31) {
            throw new IllegalArgumentException("gapCells");
        }
        return new PlatformJumpCapabilities(gapCells + 1, 0, 0, 1);
    }

    /** Names each limit. Every unstated limit is zero. */
    public static Builder builder() {
        return new Builder();
    }

    public double maxHorizontalDistance() {
        return maxHorizontalDistance;
    }

    public double maxRise() {
        return maxRise;
    }

    public double maxDrop() {
        return maxDrop;
    }

    public double apexClearance() {
        return apexClearance;
    }

    public PlatformJumpCapabilities withRise(double rise) {
        return new PlatformJumpCapabilities(
                maxHorizontalDistance, rise, maxDrop, apexClearance);
    }

    public PlatformJumpCapabilities withDrop(double drop) {
        return new PlatformJumpCapabilities(
                maxHorizontalDistance, maxRise, drop, apexClearance);
    }

    public PlatformJumpCapabilities withApexClearance(double clearance) {
        return new PlatformJumpCapabilities(
                maxHorizontalDistance, maxRise, maxDrop, clearance);
    }

    public boolean enabled() {
        return maxHorizontalDistance >= 2;
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (!(object instanceof PlatformJumpCapabilities other)) return false;
        return Double.compare(maxHorizontalDistance,
                        other.maxHorizontalDistance) == 0
                && Double.compare(maxRise, other.maxRise) == 0
                && Double.compare(maxDrop, other.maxDrop) == 0
                && Double.compare(apexClearance, other.apexClearance) == 0;
    }

    @Override
    public int hashCode() {
        int result = Double.hashCode(maxHorizontalDistance);
        result = 31 * result + Double.hashCode(maxRise);
        result = 31 * result + Double.hashCode(maxDrop);
        return 31 * result + Double.hashCode(apexClearance);
    }

    @Override
    public String toString() {
        return "PlatformJumpCapabilities[maxHorizontalDistance="
                + maxHorizontalDistance + ", maxRise=" + maxRise
                + ", maxDrop=" + maxDrop + ", apexClearance=" + apexClearance
                + "]";
    }

    /** Every limit defaults to zero, which is {@link #DISABLED}. */
    public static final class Builder {
        private double maxHorizontalDistance;
        private double maxRise;
        private double maxDrop;
        private double apexClearance;

        private Builder() {
        }

        public Builder maxHorizontalDistance(double value) {
            maxHorizontalDistance = value;
            return this;
        }

        public Builder maxRise(double value) {
            maxRise = value;
            return this;
        }

        public Builder maxDrop(double value) {
            maxDrop = value;
            return this;
        }

        public Builder apexClearance(double value) {
            apexClearance = value;
            return this;
        }

        public PlatformJumpCapabilities build() {
            return new PlatformJumpCapabilities(maxHorizontalDistance,
                    maxRise, maxDrop, apexClearance);
        }
    }
}
