package ca.atlasengine.pathfinding.profile;

import ca.atlasengine.pathfinding.terrain.BuiltinMobProfiles;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.entity.EntityType;

import java.util.Map;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.metadata.AgeableMobMeta;

/**
 * Selects an evaluator and behavior profile for baseline entity families.
 */
public final class BuiltinNavigationProfiles {
    private BuiltinNavigationProfiles() {
    }

    private static final Map<EntityType, NavigationMode> MODES = Map.ofEntries(
            Map.entry(EntityType.SPIDER, NavigationMode.WALL_CLIMBER),
            Map.entry(EntityType.CAVE_SPIDER, NavigationMode.WALL_CLIMBER),
            Map.entry(EntityType.COD, NavigationMode.WATER),
            Map.entry(EntityType.SALMON, NavigationMode.WATER),
            Map.entry(EntityType.PUFFERFISH, NavigationMode.WATER),
            Map.entry(EntityType.TROPICAL_FISH, NavigationMode.WATER),
            Map.entry(EntityType.GUARDIAN, NavigationMode.WATER),
            Map.entry(EntityType.ELDER_GUARDIAN, NavigationMode.WATER),
            Map.entry(EntityType.DOLPHIN, NavigationMode.WATER),
            Map.entry(EntityType.TADPOLE, NavigationMode.WATER),
            Map.entry(EntityType.NAUTILUS, NavigationMode.WATER),
            Map.entry(EntityType.ZOMBIE_NAUTILUS, NavigationMode.WATER),
            Map.entry(EntityType.FROG, NavigationMode.AMPHIBIOUS),
            Map.entry(EntityType.AXOLOTL, NavigationMode.AMPHIBIOUS),
            Map.entry(EntityType.TURTLE, NavigationMode.AMPHIBIOUS),
            Map.entry(EntityType.DROWNED, NavigationMode.AMPHIBIOUS),
            Map.entry(EntityType.BEE, NavigationMode.FLYING),
            Map.entry(EntityType.ALLAY, NavigationMode.FLYING),
            Map.entry(EntityType.PARROT, NavigationMode.FLYING),
            Map.entry(EntityType.WITHER, NavigationMode.FLYING));

    private static final Map<EntityType, Double> SEARCH_RANGES = Map.of(
            EntityType.FOX, 32.0,
            EntityType.LLAMA, 40.0,
            EntityType.TRADER_LLAMA, 40.0,
            EntityType.ALLAY, 48.0,
            EntityType.BEE, 48.0,
            EntityType.COPPER_GOLEM, 48.0,
            EntityType.VILLAGER, 48.0);

    public static NavigationProfile forEntityType(EntityType type) {
        if (type == null) throw new IllegalArgumentException("type");
        MobTraversalProfile malus = BuiltinMobProfiles.forEntityType(type);
        NavigationMode navigation =
                MODES.getOrDefault(type, NavigationMode.GROUND);
        GroundCapabilities capabilities;
        if (type == EntityType.CAMEL || type == EntityType.CAMEL_HUSK) {
            capabilities = GroundCapabilities.builder()
                    .maxStepHeight(1.5)
                    .build();
        } else if (type == EntityType.WARDEN) {
            capabilities = GroundCapabilities.builder()
                    .horizontalEdgeCost(true)
                    .build();
        } else {
            capabilities = GroundCapabilities.STANDARD;
        }
        return NavigationProfile.builder(navigation, malus, capabilities)
                .allowBreaching(type == EntityType.DOLPHIN)
                .prefersShallowWater(type == EntityType.FROG)
                .directPathIgnoresFluids(
                        navigation == NavigationMode.WATER
                                || navigation == NavigationMode.AMPHIBIOUS)
                .build();
    }

    /**
     * Selects a profile using live metadata when the navigation family is not
     * determined by entity type alone.
     */
    public static NavigationProfile forEntity(Entity entity) {
        if (entity == null) throw new IllegalArgumentException("entity");
        NavigationProfile base = forEntityType(entity.getEntityType());
        if (entity.getEntityType() != EntityType.HAPPY_GHAST
                || !(entity.getEntityMeta() instanceof AgeableMobMeta age)
                || !age.isBaby()) {
            return base;
        }
        return base.withNavigationType(NavigationMode.FLYING);
    }

    /**
     * Minimum terrain-search range configured by the entity's navigation.
     * The live follow-range attribute may increase this value.
     */
    public static double requiredPathLength(Entity entity) {
        if (entity == null) throw new IllegalArgumentException("entity");
        EntityType type = entity.getEntityType();
        if (type == EntityType.HAPPY_GHAST
                && entity.getEntityMeta() instanceof AgeableMobMeta age
                && age.isBaby()) {
            return 48;
        }
        return SEARCH_RANGES.getOrDefault(type, 16.0);
    }

    /**
     * Applies terrain costs that depend on the entity's live state for one
     * immutable asynchronous request.
     */
    public static NavigationProfile withLiveTerrainState(
            NavigationProfile profile, EntityType entityType,
            boolean onFire, boolean inWater) {
        if (profile == null || entityType == null) {
            throw new IllegalArgumentException("profile state");
        }
        if (entityType != EntityType.SNIFFER || (!onFire && !inWater)) {
            return profile;
        }
        MobTraversalProfile base = profile.mobProfile();
        MobTraversalProfile snapshot =
                MobTraversalProfile.builder(base.name() + "_request")
                        .from(base)
                        .malus(TerrainType.WATER, 0)
                        .build();
        return profile.withMobProfile(snapshot);
    }

    /**
     * Applies terrain-cost inheritance for a rider whose vehicle supplies its
     * traversal behavior.
     */
    public static NavigationProfile withVehicleTerrainProfile(
            NavigationProfile riderProfile, EntityType vehicleType) {
        if (riderProfile == null) throw new IllegalArgumentException("profile");
        if (vehicleType != EntityType.STRIDER) return riderProfile;
        return riderProfile.withMobProfile(MobTraversalProfile.STRIDER);
    }
}
