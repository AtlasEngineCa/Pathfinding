package ca.atlasengine.pathfinding.profile;

/**
 * Physical movement abilities. These are never inferred from terrain.
 *
 * <p>Not a record: {@code maxStepHeight} and {@code maxFallDistance} are both
 * distances and {@code allowDiagonal} and {@code horizontalEdgeCost} are both
 * flags, so a positional constructor would let either pair be transposed into
 * a mob that steps as high as it may fall. Reach an instance through
 * {@link #STANDARD} or {@link #builder()}.</p>
 */
public final class GroundCapabilities {
    /** Bounds the vertical scans performed for every expanded ground node. */
    public static final double MAX_STEP_HEIGHT = 16;
    public static final double MAX_FALL_DISTANCE = 1024;

    public static final GroundCapabilities STANDARD =
            new GroundCapabilities(1.0, 5.0, true, false,
                    PlatformJumpCapabilities.DISABLED,
                    ClimbableCapabilities.DISABLED);

    private final double maxStepHeight;
    private final double maxFallDistance;
    private final boolean allowDiagonal;
    private final boolean horizontalEdgeCost;
    private final PlatformJumpCapabilities platformJump;
    private final ClimbableCapabilities climbables;

    private GroundCapabilities(double maxStepHeight, double maxFallDistance,
                               boolean allowDiagonal,
                               boolean horizontalEdgeCost,
                               PlatformJumpCapabilities platformJump,
                               ClimbableCapabilities climbables) {
        if (!Double.isFinite(maxStepHeight) || maxStepHeight < 0
                || maxStepHeight > MAX_STEP_HEIGHT
                || !Double.isFinite(maxFallDistance) || maxFallDistance < 0
                || maxFallDistance > MAX_FALL_DISTANCE
                || platformJump == null || climbables == null) {
            throw new IllegalArgumentException("Invalid ground capabilities");
        }
        this.maxStepHeight = maxStepHeight;
        this.maxFallDistance = maxFallDistance;
        this.allowDiagonal = allowDiagonal;
        this.horizontalEdgeCost = horizontalEdgeCost;
        this.platformJump = platformJump;
        this.climbables = climbables;
    }

    /** Starts from {@link #STANDARD} so only deliberate changes are stated. */
    public static Builder builder() {
        return new Builder();
    }

    public double maxStepHeight() {
        return maxStepHeight;
    }

    public double maxFallDistance() {
        return maxFallDistance;
    }

    public boolean allowDiagonal() {
        return allowDiagonal;
    }

    public boolean horizontalEdgeCost() {
        return horizontalEdgeCost;
    }

    public PlatformJumpCapabilities platformJump() {
        return platformJump;
    }

    public ClimbableCapabilities climbables() {
        return climbables;
    }

    public GroundCapabilities withHorizontalEdgeCost(boolean value) {
        return new GroundCapabilities(maxStepHeight, maxFallDistance,
                allowDiagonal, value, platformJump, climbables);
    }

    public GroundCapabilities withPlatformJump(
            PlatformJumpCapabilities jump) {
        return new GroundCapabilities(maxStepHeight, maxFallDistance,
                allowDiagonal, horizontalEdgeCost, jump, climbables);
    }

    public GroundCapabilities withClimbables(
            ClimbableCapabilities capabilities) {
        return new GroundCapabilities(maxStepHeight, maxFallDistance,
                allowDiagonal, horizontalEdgeCost, platformJump, capabilities);
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (!(object instanceof GroundCapabilities other)) return false;
        return Double.compare(maxStepHeight, other.maxStepHeight) == 0
                && Double.compare(maxFallDistance, other.maxFallDistance) == 0
                && allowDiagonal == other.allowDiagonal
                && horizontalEdgeCost == other.horizontalEdgeCost
                && platformJump.equals(other.platformJump)
                && climbables.equals(other.climbables);
    }

    @Override
    public int hashCode() {
        int result = Double.hashCode(maxStepHeight);
        result = 31 * result + Double.hashCode(maxFallDistance);
        result = 31 * result + Boolean.hashCode(allowDiagonal);
        result = 31 * result + Boolean.hashCode(horizontalEdgeCost);
        result = 31 * result + platformJump.hashCode();
        return 31 * result + climbables.hashCode();
    }

    @Override
    public String toString() {
        return "GroundCapabilities[maxStepHeight=" + maxStepHeight
                + ", maxFallDistance=" + maxFallDistance
                + ", allowDiagonal=" + allowDiagonal
                + ", horizontalEdgeCost=" + horizontalEdgeCost
                + ", platformJump=" + platformJump
                + ", climbables=" + climbables + "]";
    }

    /** Starts from {@link GroundCapabilities#STANDARD}. */
    public static final class Builder {
        private double maxStepHeight = STANDARD.maxStepHeight;
        private double maxFallDistance = STANDARD.maxFallDistance;
        private boolean allowDiagonal = STANDARD.allowDiagonal;
        private boolean horizontalEdgeCost = STANDARD.horizontalEdgeCost;
        private PlatformJumpCapabilities platformJump = STANDARD.platformJump;
        private ClimbableCapabilities climbables = STANDARD.climbables;

        private Builder() {
        }

        public Builder maxStepHeight(double value) {
            maxStepHeight = value;
            return this;
        }

        public Builder maxFallDistance(double value) {
            maxFallDistance = value;
            return this;
        }

        public Builder allowDiagonal(boolean value) {
            allowDiagonal = value;
            return this;
        }

        public Builder horizontalEdgeCost(boolean value) {
            horizontalEdgeCost = value;
            return this;
        }

        public Builder platformJump(PlatformJumpCapabilities value) {
            platformJump = value;
            return this;
        }

        public Builder climbables(ClimbableCapabilities value) {
            climbables = value;
            return this;
        }

        public GroundCapabilities build() {
            return new GroundCapabilities(maxStepHeight, maxFallDistance,
                    allowDiagonal, horizontalEdgeCost, platformJump,
                    climbables);
        }
    }
}
