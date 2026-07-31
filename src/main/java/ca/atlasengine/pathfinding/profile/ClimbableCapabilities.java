package ca.atlasengine.pathfinding.profile;

/**
 * Optional route-planning support for tagged climbable blocks.
 *
 * <p>Minecraft living-entity physics understands climbables, but its ordinary
 * ground node evaluator does not add vertical ladder edges. Consequently the
 * Default profiles leave this disabled. Integrations may opt in when
 * they intentionally want mobs to plan routes through ladders, vines, and
 * scaffolding.</p>
 *
 * <p>Not a record: three consecutive booleans would let a positional
 * constructor silently swap ascending for descending. Reach an instance
 * through the direction factories below.</p>
 */
public final class ClimbableCapabilities {
    public static final ClimbableCapabilities DISABLED =
            new ClimbableCapabilities(false, false, false, 1);
    public static final ClimbableCapabilities STANDARD =
            new ClimbableCapabilities(true, true, true, 1);

    private final boolean enabled;
    private final boolean allowAscending;
    private final boolean allowDescending;
    private final double verticalCostMultiplier;

    private ClimbableCapabilities(boolean enabled, boolean allowAscending,
                                  boolean allowDescending,
                                  double verticalCostMultiplier) {
        if (!Double.isFinite(verticalCostMultiplier)
                || verticalCostMultiplier <= 0) {
            throw new IllegalArgumentException("verticalCostMultiplier");
        }
        if (!enabled && (allowAscending || allowDescending)) {
            throw new IllegalArgumentException(
                    "disabled climbables cannot enable a direction");
        }
        this.enabled = enabled;
        this.allowAscending = allowAscending;
        this.allowDescending = allowDescending;
        this.verticalCostMultiplier = verticalCostMultiplier;
    }

    /** Ascends and descends, at the given vertical edge-cost multiplier. */
    public static ClimbableCapabilities bothDirections(double verticalCost) {
        return new ClimbableCapabilities(true, true, true, verticalCost);
    }

    /** Climbs up only, so a planned route never descends a ladder. */
    public static ClimbableCapabilities ascendOnly(double verticalCost) {
        return new ClimbableCapabilities(true, true, false, verticalCost);
    }

    /** Climbs down only. */
    public static ClimbableCapabilities descendOnly(double verticalCost) {
        return new ClimbableCapabilities(true, false, true, verticalCost);
    }

    /** Names each flag, for combinations the factories above do not cover. */
    public static Builder builder() {
        return new Builder();
    }

    public boolean enabled() {
        return enabled;
    }

    public boolean allowAscending() {
        return allowAscending;
    }

    public boolean allowDescending() {
        return allowDescending;
    }

    public double verticalCostMultiplier() {
        return verticalCostMultiplier;
    }

    public ClimbableCapabilities withVerticalCostMultiplier(double value) {
        return new ClimbableCapabilities(
                enabled, allowAscending, allowDescending, value);
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (!(object instanceof ClimbableCapabilities other)) return false;
        return enabled == other.enabled
                && allowAscending == other.allowAscending
                && allowDescending == other.allowDescending
                && Double.compare(verticalCostMultiplier,
                        other.verticalCostMultiplier) == 0;
    }

    @Override
    public int hashCode() {
        int result = Boolean.hashCode(enabled);
        result = 31 * result + Boolean.hashCode(allowAscending);
        result = 31 * result + Boolean.hashCode(allowDescending);
        return 31 * result + Double.hashCode(verticalCostMultiplier);
    }

    @Override
    public String toString() {
        return "ClimbableCapabilities[enabled=" + enabled
                + ", allowAscending=" + allowAscending
                + ", allowDescending=" + allowDescending
                + ", verticalCostMultiplier=" + verticalCostMultiplier + "]";
    }

    /** Disabled with a multiplier of one unless stated otherwise. */
    public static final class Builder {
        private boolean enabled;
        private boolean allowAscending;
        private boolean allowDescending;
        private double verticalCostMultiplier = 1;

        private Builder() {
        }

        public Builder enabled(boolean value) {
            enabled = value;
            return this;
        }

        public Builder allowAscending(boolean value) {
            allowAscending = value;
            return this;
        }

        public Builder allowDescending(boolean value) {
            allowDescending = value;
            return this;
        }

        public Builder verticalCostMultiplier(double value) {
            verticalCostMultiplier = value;
            return this;
        }

        public ClimbableCapabilities build() {
            return new ClimbableCapabilities(enabled, allowAscending,
                    allowDescending, verticalCostMultiplier);
        }
    }
}
