package ca.atlasengine.pathfinding.adaptive;

import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.NavigationStrategy;

/**
 * One shared-mesh planning request: the search itself plus the routing
 * identities the mesh needs and an ordinary request does not.
 *
 * <p>Use {@link #builder(NavigationRequest)} so the two opaque identities, the
 * world revision, and the tick are named at the call site instead of being
 * four positional values in a row.</p>
 */
public record SharedMeshRequest(
        Object actorKey,
        Object targetKey,
        long worldRevision,
        long currentTick,
        NavigationStrategy strategy,
        NavigationRequest search) {

    public SharedMeshRequest {
        if (actorKey == null || targetKey == null || strategy == null
                || search == null || currentTick < 0) {
            throw new IllegalArgumentException("invalid shared mesh request");
        }
    }

    public static Builder builder(NavigationRequest search) {
        if (search == null) throw new IllegalArgumentException("search");
        return new Builder(search);
    }

    public static final class Builder {
        private final NavigationRequest search;
        private Object actorKey;
        private Object targetKey;
        private long worldRevision;
        private long currentTick;
        private NavigationStrategy strategy = NavigationStrategy.PREFER_SHARED;

        private Builder(NavigationRequest search) {
            this.search = search;
        }

        /**
         * The stable identity of the navigating entity, normally its UUID.
         * Requesting again for the same actor releases its previous
         * membership.
         */
        public Builder actor(Object key) {
            actorKey = key;
            return this;
        }

        /**
         * The stable identity of the pursued entity or fixed objective. The
         * destination's own block cell is tracked separately, so a target that
         * walks keeps one identity and gains a second target field.
         */
        public Builder target(Object key) {
            targetKey = key;
            return this;
        }

        /**
         * An integration-owned counter that must change whenever retained
         * collision topology becomes stale. Retained routes of an older
         * revision are dropped rather than replayed.
         */
        public Builder worldRevision(long value) {
            worldRevision = value;
            return this;
        }

        /** The owning world or server tick, used for expiry and hysteresis. */
        public Builder currentTick(long value) {
            currentTick = value;
            return this;
        }

        /**
         * Defaults to {@link NavigationStrategy#PREFER_SHARED}. Set {@link
         * NavigationStrategy#INDIVIDUAL_ONLY} to keep this request off the
         * mesh entirely.
         */
        public Builder strategy(NavigationStrategy value) {
            strategy = value;
            return this;
        }

        public SharedMeshRequest build() {
            return new SharedMeshRequest(actorKey, targetKey, worldRevision,
                    currentTick, strategy, search);
        }
    }
}
