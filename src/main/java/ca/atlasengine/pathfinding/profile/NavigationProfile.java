package ca.atlasengine.pathfinding.profile;

import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;

/**
 * Complete high-level navigation selection for an entity family.
 *
 * <p>Not a record: five consecutive booleans would read as
 * {@code (false, false, true, false, false)} at a positional call site, where
 * nothing but argument order separates breaching from sun avoidance. Reach an
 * instance through {@link #builder(NavigationMode, MobTraversalProfile,
 * GroundCapabilities)}.</p>
 *
 * <p>Equality is component-wise and load-bearing: adaptive regions key on the
 * whole profile, so two mobs share a computed route only while their profiles
 * compare equal.</p>
 */
public final class NavigationProfile {
    private final NavigationMode mode;
    private final MobTraversalProfile mobProfile;
    private final GroundCapabilities groundCapabilities;
    private final boolean allowBreaching;
    private final boolean prefersShallowWater;
    private final boolean avoidSun;
    private final boolean directPathIgnoresFluids;
    private final boolean pathToTargetsBelowSurface;

    private NavigationProfile(NavigationMode mode,
                              MobTraversalProfile mobProfile,
                              GroundCapabilities groundCapabilities,
                              boolean allowBreaching,
                              boolean prefersShallowWater,
                              boolean avoidSun,
                              boolean directPathIgnoresFluids,
                              boolean pathToTargetsBelowSurface) {
        if (mode == null || mobProfile == null
                || groundCapabilities == null) {
            throw new IllegalArgumentException("profile fields");
        }
        this.mode = mode;
        this.mobProfile = mobProfile;
        this.groundCapabilities = groundCapabilities;
        this.allowBreaching = allowBreaching;
        this.prefersShallowWater = prefersShallowWater;
        this.avoidSun = avoidSun;
        this.directPathIgnoresFluids = directPathIgnoresFluids;
        this.pathToTargetsBelowSurface = pathToTargetsBelowSurface;
    }

    /**
     * The three required pieces are arguments because they are distinct types;
     * every optional flag is named and defaults to false, so a built profile
     * states exactly what it turns on.
     */
    public static Builder builder(NavigationMode mode,
                                  MobTraversalProfile mobProfile,
                                  GroundCapabilities groundCapabilities) {
        return new Builder(mode, mobProfile, groundCapabilities);
    }

    public NavigationMode mode() {
        return mode;
    }

    public MobTraversalProfile mobProfile() {
        return mobProfile;
    }

    public GroundCapabilities groundCapabilities() {
        return groundCapabilities;
    }

    public boolean allowBreaching() {
        return allowBreaching;
    }

    public boolean prefersShallowWater() {
        return prefersShallowWater;
    }

    public boolean avoidSun() {
        return avoidSun;
    }

    public boolean directPathIgnoresFluids() {
        return directPathIgnoresFluids;
    }

    public boolean pathToTargetsBelowSurface() {
        return pathToTargetsBelowSurface;
    }

    public NavigationProfile withNavigationType(NavigationMode type) {
        return new NavigationProfile(type, mobProfile,
                groundCapabilities, allowBreaching,
                prefersShallowWater, avoidSun,
                directPathIgnoresFluids, pathToTargetsBelowSurface);
    }

    public NavigationProfile withMobProfile(MobTraversalProfile profile) {
        return new NavigationProfile(mode, profile,
                groundCapabilities, allowBreaching,
                prefersShallowWater, avoidSun,
                directPathIgnoresFluids, pathToTargetsBelowSurface);
    }

    public NavigationProfile withGroundCapabilities(
            GroundCapabilities capabilities) {
        return new NavigationProfile(mode, mobProfile,
                capabilities, allowBreaching,
                prefersShallowWater, avoidSun,
                directPathIgnoresFluids, pathToTargetsBelowSurface);
    }

    public NavigationProfile withAvoidSun(boolean value) {
        return new NavigationProfile(mode, mobProfile,
                groundCapabilities, allowBreaching,
                prefersShallowWater, value,
                directPathIgnoresFluids, pathToTargetsBelowSurface);
    }

    public NavigationProfile withPathToTargetsBelowSurface(boolean value) {
        return new NavigationProfile(mode, mobProfile,
                groundCapabilities, allowBreaching,
                prefersShallowWater, avoidSun,
                directPathIgnoresFluids, value);
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (!(object instanceof NavigationProfile other)) return false;
        return mode == other.mode
                && mobProfile.equals(other.mobProfile)
                && groundCapabilities.equals(other.groundCapabilities)
                && allowBreaching == other.allowBreaching
                && prefersShallowWater == other.prefersShallowWater
                && avoidSun == other.avoidSun
                && directPathIgnoresFluids == other.directPathIgnoresFluids
                && pathToTargetsBelowSurface
                        == other.pathToTargetsBelowSurface;
    }

    @Override
    public int hashCode() {
        int result = mode.hashCode();
        result = 31 * result + mobProfile.hashCode();
        result = 31 * result + groundCapabilities.hashCode();
        result = 31 * result + Boolean.hashCode(allowBreaching);
        result = 31 * result + Boolean.hashCode(prefersShallowWater);
        result = 31 * result + Boolean.hashCode(avoidSun);
        result = 31 * result + Boolean.hashCode(directPathIgnoresFluids);
        return 31 * result + Boolean.hashCode(pathToTargetsBelowSurface);
    }

    @Override
    public String toString() {
        return "NavigationProfile[mode=" + mode
                + ", mobProfile=" + mobProfile
                + ", groundCapabilities=" + groundCapabilities
                + ", allowBreaching=" + allowBreaching
                + ", prefersShallowWater=" + prefersShallowWater
                + ", avoidSun=" + avoidSun
                + ", directPathIgnoresFluids=" + directPathIgnoresFluids
                + ", pathToTargetsBelowSurface=" + pathToTargetsBelowSurface
                + "]";
    }

    /** Every flag defaults to false. */
    public static final class Builder {
        private final NavigationMode mode;
        private final MobTraversalProfile mobProfile;
        private final GroundCapabilities groundCapabilities;
        private boolean allowBreaching;
        private boolean prefersShallowWater;
        private boolean avoidSun;
        private boolean directPathIgnoresFluids;
        private boolean pathToTargetsBelowSurface;

        private Builder(NavigationMode mode,
                        MobTraversalProfile mobProfile,
                        GroundCapabilities groundCapabilities) {
            this.mode = mode;
            this.mobProfile = mobProfile;
            this.groundCapabilities = groundCapabilities;
        }

        public Builder allowBreaching(boolean value) {
            allowBreaching = value;
            return this;
        }

        public Builder prefersShallowWater(boolean value) {
            prefersShallowWater = value;
            return this;
        }

        public Builder avoidSun(boolean value) {
            avoidSun = value;
            return this;
        }

        public Builder directPathIgnoresFluids(boolean value) {
            directPathIgnoresFluids = value;
            return this;
        }

        public Builder pathToTargetsBelowSurface(boolean value) {
            pathToTargetsBelowSurface = value;
            return this;
        }

        public NavigationProfile build() {
            return new NavigationProfile(mode, mobProfile,
                groundCapabilities, allowBreaching,
                prefersShallowWater, avoidSun,
                directPathIgnoresFluids, pathToTargetsBelowSurface);
        }
    }
}
