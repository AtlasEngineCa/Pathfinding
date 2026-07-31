package ca.atlasengine.pathfinding.adaptive;

/** Bounded lifecycle and promotion policy for adaptive shared navigation. */
public record SharedMeshPolicy(
        int promotionRequests,
        int retentionRequests,
        int regionSize,
        int verticalRegionSize,
        long targetIdleTicks,
        long regionIdleTicks,
        int maximumRegions,
        int maximumNodesPerRegion,
        int maximumTargetsPerRegion
) {
    public static final SharedMeshPolicy DEFAULT = builder().build();

    public SharedMeshPolicy {
        if (promotionRequests < 2 || retentionRequests < 1
                || retentionRequests >= promotionRequests
                || regionSize < 8 || verticalRegionSize < 8
                || targetIdleTicks < 1 || regionIdleTicks < targetIdleTicks
                || maximumRegions < 1 || maximumNodesPerRegion < 2
                || maximumTargetsPerRegion < 1) {
            throw new IllegalArgumentException("invalid adaptive navigation policy");
        }
    }

    public static Builder builder() {
        return new Builder();
    }

    public static final class Builder {
        private int promotionRequests = 16;
        private int retentionRequests = 4;
        private int regionSize = 64;
        private int verticalRegionSize = 64;
        private long targetIdleTicks = 200;
        private long regionIdleTicks = 600;
        private int maximumRegions = 128;
        private int maximumNodesPerRegion = 2_048;
        private int maximumTargetsPerRegion = 32;

        private Builder() {
        }

        public Builder promotionRequests(int value) {
            promotionRequests = value;
            return this;
        }

        public Builder retentionRequests(int value) {
            retentionRequests = value;
            return this;
        }

        public Builder regionSize(int value) {
            regionSize = value;
            return this;
        }

        public Builder verticalRegionSize(int value) {
            verticalRegionSize = value;
            return this;
        }

        public Builder targetIdleTicks(long value) {
            targetIdleTicks = value;
            return this;
        }

        public Builder regionIdleTicks(long value) {
            regionIdleTicks = value;
            return this;
        }

        public Builder maximumRegions(int value) {
            maximumRegions = value;
            return this;
        }

        public Builder maximumNodesPerRegion(int value) {
            maximumNodesPerRegion = value;
            return this;
        }

        public Builder maximumTargetsPerRegion(int value) {
            maximumTargetsPerRegion = value;
            return this;
        }

        public SharedMeshPolicy build() {
            return new SharedMeshPolicy(
                    promotionRequests, retentionRequests, regionSize,
                    verticalRegionSize, targetIdleTicks, regionIdleTicks,
                    maximumRegions, maximumNodesPerRegion,
                    maximumTargetsPerRegion);
        }
    }
}
