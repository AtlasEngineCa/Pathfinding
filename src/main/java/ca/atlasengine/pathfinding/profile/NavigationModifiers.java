package ca.atlasengine.pathfinding.profile;

import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainType;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;

/**
 * Immutable request-scoped changes to an entity's normal navigation profile.
 *
 * <p>Use this for temporary goal or live-state behavior. Applying a modifier
 * creates a new profile; neither the baseline profile nor other concurrent
 * requests are mutated.</p>
 */
public final class NavigationModifiers {
    public static final NavigationModifiers NONE = builder().build();

    private final Map<TerrainType, Double> terrainCosts;
    private final Boolean canPassDoors;
    private final Boolean canOpenDoors;
    private final BlockManipulationCapabilities blockManipulation;
    private final Boolean canFloat;
    private final Boolean canWalkOverFences;
    private final Boolean avoidSun;
    private final PlatformJumpCapabilities platformJump;
    private final ClimbableCapabilities climbables;
    private final Boolean pathToTargetsBelowSurface;

    private NavigationModifiers(Builder builder) {
        terrainCosts = Collections.unmodifiableMap(
                new EnumMap<>(builder.terrainCosts));
        canPassDoors = builder.canPassDoors;
        canOpenDoors = builder.canOpenDoors;
        blockManipulation = builder.blockManipulation;
        canFloat = builder.canFloat;
        canWalkOverFences = builder.canWalkOverFences;
        avoidSun = builder.avoidSun;
        platformJump = builder.platformJump;
        climbables = builder.climbables;
        pathToTargetsBelowSurface = builder.pathToTargetsBelowSurface;
    }

    public Map<TerrainType, Double> terrainCosts() {
        return terrainCosts;
    }

    public NavigationProfile applyTo(NavigationProfile base) {
        if (base == null) throw new IllegalArgumentException("base");
        if (terrainCosts.isEmpty() && canPassDoors == null
                && canOpenDoors == null && blockManipulation == null
                && canFloat == null
                && canWalkOverFences == null && avoidSun == null
                && platformJump == null && climbables == null
                && pathToTargetsBelowSurface == null) {
            return base;
        }
        MobTraversalProfile source = base.mobProfile();
        MobTraversalProfile.Builder mob = MobTraversalProfile
                .builder(source.name() + "_modified").from(source);
        terrainCosts.forEach(mob::malus);
        if (canPassDoors != null) mob.canPassDoors(canPassDoors);
        if (blockManipulation != null) {
            mob.blockManipulation(blockManipulation);
        }
        if (canOpenDoors != null) mob.canOpenDoors(canOpenDoors);
        if (canFloat != null) mob.canFloat(canFloat);
        if (canWalkOverFences != null) {
            mob.canWalkOverFences(canWalkOverFences);
        }
        NavigationProfile profile = base.withMobProfile(mob.build())
                .withGroundCapabilities(
                        applyCapabilities(base.groundCapabilities()));
        if (avoidSun != null) profile = profile.withAvoidSun(avoidSun);
        if (pathToTargetsBelowSurface != null) {
            profile = profile.withPathToTargetsBelowSurface(
                    pathToTargetsBelowSurface);
        }
        return profile;
    }

    public static Builder builder() {
        return new Builder();
    }

    private GroundCapabilities applyCapabilities(GroundCapabilities base) {
        GroundCapabilities result = platformJump == null
                ? base : base.withPlatformJump(platformJump);
        return climbables == null ? result : result.withClimbables(climbables);
    }

    public static final class Builder {
        private final EnumMap<TerrainType, Double> terrainCosts =
                new EnumMap<>(TerrainType.class);
        private Boolean canPassDoors;
        private Boolean canOpenDoors;
        private BlockManipulationCapabilities blockManipulation;
        private Boolean canFloat;
        private Boolean canWalkOverFences;
        private Boolean avoidSun;
        private PlatformJumpCapabilities platformJump;
        private ClimbableCapabilities climbables;
        private Boolean pathToTargetsBelowSurface;

        private Builder() {
        }

        public Builder terrainCost(TerrainType type, double cost) {
            if (type == null || !Double.isFinite(cost)) {
                throw new IllegalArgumentException("invalid terrain cost");
            }
            terrainCosts.put(type, cost);
            return this;
        }

        public Builder canPassDoors(boolean value) {
            canPassDoors = value;
            return this;
        }

        public Builder canOpenDoors(boolean value) {
            canOpenDoors = value;
            return this;
        }

        /**
         * Grants trapdoor or fence-gate manipulation, or closing behind, for
         * the lifetime of one request. A later {@link #canOpenDoors(boolean)}
         * still refines the door family on top of this.
         */
        public Builder blockManipulation(
                BlockManipulationCapabilities capabilities) {
            if (capabilities == null) {
                throw new IllegalArgumentException(
                        "block manipulation capabilities");
            }
            blockManipulation = capabilities;
            return this;
        }

        public Builder canFloat(boolean value) {
            canFloat = value;
            return this;
        }

        public Builder canWalkOverFences(boolean value) {
            canWalkOverFences = value;
            return this;
        }

        public Builder avoidSun(boolean value) {
            avoidSun = value;
            return this;
        }

        public Builder platformJump(
                PlatformJumpCapabilities capabilities) {
            if (capabilities == null) {
                throw new IllegalArgumentException(
                        "platform jump capabilities");
            }
            platformJump = capabilities;
            return this;
        }

        /** Enables or disables intentional routes through climbable blocks. */
        public Builder climbables(ClimbableCapabilities capabilities) {
            if (capabilities == null) {
                throw new IllegalArgumentException("climbable capabilities");
            }
            climbables = capabilities;
            return this;
        }

        /** Mirrors GroundPathNavigation's temporary buried-target switch. */
        public Builder pathToTargetsBelowSurface(boolean value) {
            pathToTargetsBelowSurface = value;
            return this;
        }

        public NavigationModifiers build() {
            return new NavigationModifiers(this);
        }
    }
}
