package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.terrain.BuiltinMobProfiles;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.entity.EntityType;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.metadata.AgeableMobMeta;
import org.junit.jupiter.api.Test;

import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Auditable expectations transcribed from the 26.2 entity constructors,
 * registerGoals methods, and navigation factories.
 */
class BuiltinProfileAuditTest {
    @Test
    void constructorMalusOverridesAreExact() {
        assertOverrides(MobTraversalProfile.ANIMAL,
                TerrainType.FIRE_IN_NEIGHBOR, 16,
                TerrainType.FIRE, -1);
        assertOverrides(MobTraversalProfile.WATER_ANIMAL,
                TerrainType.WATER, 0);
        assertOverrides(MobTraversalProfile.NAUTILUS,
                TerrainType.FIRE_IN_NEIGHBOR, 16,
                TerrainType.FIRE, -1,
                TerrainType.WATER, 0);
        assertOverrides(MobTraversalProfile.PIGLIN,
                TerrainType.FIRE_IN_NEIGHBOR, 16,
                TerrainType.FIRE, -1);
        assertOverrides(MobTraversalProfile.ZOMBIFIED_PIGLIN,
                TerrainType.LAVA, 8);
        assertOverrides(MobTraversalProfile.BLAZE,
                TerrainType.WATER, -1,
                TerrainType.LAVA, 8,
                TerrainType.FIRE_IN_NEIGHBOR, 0,
                TerrainType.FIRE, 0);
        assertOverrides(MobTraversalProfile.STRIDER,
                TerrainType.WATER, -1,
                TerrainType.LAVA, 0,
                TerrainType.FIRE_IN_NEIGHBOR, 0,
                TerrainType.FIRE, 0);
        assertOverrides(MobTraversalProfile.COPPER_GOLEM,
                TerrainType.FIRE_IN_NEIGHBOR, 16,
                TerrainType.DAMAGING_IN_NEIGHBOR, 16,
                TerrainType.FIRE, -1);
        assertOverrides(MobTraversalProfile.BREEZE,
                TerrainType.ON_TOP_OF_TRAPDOOR, -1,
                TerrainType.FIRE, -1);
    }

    @Test
    void inheritedAnimalOverridesAreNotAppliedToUnrelatedAquaticMobs() {
        assertEquals(16, MobTraversalProfile.NAUTILUS.malus(TerrainType.FIRE_IN_NEIGHBOR));
        assertEquals(8, MobTraversalProfile.WATER_ANIMAL.malus(TerrainType.FIRE_IN_NEIGHBOR));
        assertEquals(-1, MobTraversalProfile.NAUTILUS.malus(TerrainType.FIRE));
        assertEquals(16, MobTraversalProfile.WATER_ANIMAL.malus(TerrainType.FIRE));
        assertFalse(MobTraversalProfile.WATER_ANIMAL.canPassDoors());
        assertFalse(MobTraversalProfile.NAUTILUS.canPassDoors());
    }

    @Test
    void floatGoalCapabilityIsOnlyPresentForMobsThatInstallIt() {
        assertFalse(MobTraversalProfile.ANIMAL.canFloat());
        assertTrue(MobTraversalProfile.FLOATING_ANIMAL.canFloat());
        assertFalse(MobTraversalProfile.FROG.canFloat());
        assertFalse(MobTraversalProfile.TURTLE.canFloat());
        assertFalse(MobTraversalProfile.AXOLOTL.canFloat());
        assertTrue(MobTraversalProfile.FOX.canFloat());
        assertTrue(MobTraversalProfile.BEE.canFloat());
        assertTrue(MobTraversalProfile.WOLF.canFloat());
        assertTrue(MobTraversalProfile.CHICKEN.canFloat());
        assertTrue(MobTraversalProfile.ALLAY.canFloat());
        assertTrue(MobTraversalProfile.WITHER.canFloat());
        assertTrue(MobTraversalProfile.VEX.canFloat());
        assertTrue(MobTraversalProfile.HAPPY_GHAST.canFloat());
    }

    @Test
    void everyStaticDoorAndFenceAssignmentIsRepresented() {
        assertTrue(MobTraversalProfile.VILLAGER.canOpenDoors());
        assertTrue(MobTraversalProfile.PIGLIN.canOpenDoors());
        assertTrue(MobTraversalProfile.COPPER_GOLEM.canOpenDoors());
        assertFalse(MobTraversalProfile.DEFAULT.canOpenDoors());
        assertTrue(MobTraversalProfile.CAMEL.canWalkOverFences());
        assertFalse(MobTraversalProfile.ANIMAL.canWalkOverFences());
    }

    @Test
    void allSpecializedEntityMappingsMatchExpectedBehaviorFamilies() {
        assertProfile(MobTraversalProfile.FLOATING_ANIMAL,
                EntityType.COW, EntityType.SHEEP, EntityType.PIG, EntityType.HORSE,
                EntityType.DONKEY, EntityType.MULE, EntityType.RABBIT,
                EntityType.ARMADILLO, EntityType.LLAMA, EntityType.TRADER_LLAMA,
                EntityType.MOOSHROOM, EntityType.CAT, EntityType.OCELOT,
                EntityType.PANDA, EntityType.POLAR_BEAR, EntityType.SKELETON_HORSE,
                EntityType.ZOMBIE_HORSE);
        assertProfile(MobTraversalProfile.ANIMAL, EntityType.HOGLIN);
        assertProfile(MobTraversalProfile.CAMEL, EntityType.CAMEL, EntityType.CAMEL_HUSK);
        assertProfile(MobTraversalProfile.PIGLIN, EntityType.PIGLIN, EntityType.PIGLIN_BRUTE);
        assertProfile(MobTraversalProfile.NAUTILUS, EntityType.NAUTILUS, EntityType.ZOMBIE_NAUTILUS);
        assertProfile(MobTraversalProfile.FLOATING_DEFAULT,
                EntityType.SLIME, EntityType.MAGMA_CUBE, EntityType.SULFUR_CUBE,
                EntityType.SPIDER, EntityType.CAVE_SPIDER, EntityType.CREEPER);
        assertProfile(MobTraversalProfile.ALLAY, EntityType.ALLAY);
        assertProfile(MobTraversalProfile.WITHER, EntityType.WITHER);
        assertProfile(MobTraversalProfile.VEX, EntityType.VEX);
        assertProfile(MobTraversalProfile.HAPPY_GHAST, EntityType.HAPPY_GHAST);
    }

    @Test
    void navigationFactoryFlagsAndFamiliesAreExactForAuditedMobs() {
        assertFalse(BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE)
                .groundCapabilities().climbables().enabled());
        assertFalse(BuiltinNavigationProfiles.forEntityType(EntityType.HORSE)
                .groundCapabilities().climbables().enabled(),
                "horses override LivingEntity.onClimbable to false");
        assertFalse(BuiltinNavigationProfiles.forEntityType(EntityType.SPIDER)
                .groundCapabilities().climbables().enabled(),
                "spiders use collision wall climbing, not ladder graph edges");
        assertEquals(NavigationMode.WATER,
                BuiltinNavigationProfiles.forEntityType(EntityType.NAUTILUS).mode());
        assertEquals(NavigationMode.GROUND,
                BuiltinNavigationProfiles.forEntityType(EntityType.SQUID).mode(),
                "Squid inherits Mob's ground navigation; its swim movement is not path navigation");
        assertEquals(NavigationMode.GROUND,
                BuiltinNavigationProfiles.forEntityType(EntityType.GLOW_SQUID).mode(),
                "Glow squid inherits squid's non-navigation aquatic movement");
        assertEquals(NavigationMode.FLYING,
                BuiltinNavigationProfiles.forEntityType(EntityType.ALLAY).mode());
        assertEquals(NavigationMode.GROUND,
                BuiltinNavigationProfiles.forEntityType(EntityType.VEX).mode(),
                "Vex flies through its move control but does not install FlyingPathNavigation");
        assertFalse(BuiltinNavigationProfiles.forEntityType(EntityType.COD)
                .mobProfile().canPassDoors());
        assertTrue(BuiltinNavigationProfiles.forEntityType(EntityType.COD)
                .directPathIgnoresFluids());
        assertFalse(BuiltinNavigationProfiles.forEntityType(EntityType.BEE)
                .directPathIgnoresFluids());

        NavigationProfile camelHusk =
                BuiltinNavigationProfiles.forEntityType(EntityType.CAMEL_HUSK);
        assertEquals(1.5, camelHusk.groundCapabilities().maxStepHeight());
        assertTrue(camelHusk.mobProfile().canWalkOverFences());

        assertEquals(NavigationMode.GROUND,
                BuiltinNavigationProfiles.forEntityType(EntityType.HAPPY_GHAST)
                        .mode(),
                "type-only selection represents the default adult state");
        EntityCreature ghast = new EntityCreature(EntityType.HAPPY_GHAST);
        assertEquals(NavigationMode.GROUND,
                BuiltinNavigationProfiles.forEntity(ghast).mode());
        assertFalse(BuiltinNavigationProfiles.forEntity(ghast)
                .directPathIgnoresFluids());
        ((AgeableMobMeta) ghast.getEntityMeta()).setBaby(true);
        assertEquals(NavigationMode.FLYING,
                BuiltinNavigationProfiles.forEntity(ghast).mode());
        assertFalse(BuiltinNavigationProfiles.forEntity(ghast)
                .directPathIgnoresFluids());
        assertEquals(48, BuiltinNavigationProfiles.requiredPathLength(ghast));
        assertEquals(48, BuiltinNavigationProfiles.requiredPathLength(
                new EntityCreature(EntityType.ALLAY)));
        assertEquals(48, BuiltinNavigationProfiles.requiredPathLength(
                new EntityCreature(EntityType.BEE)));
        assertEquals(48, BuiltinNavigationProfiles.requiredPathLength(
                new EntityCreature(EntityType.COPPER_GOLEM)));
        assertEquals(48, BuiltinNavigationProfiles.requiredPathLength(
                new EntityCreature(EntityType.VILLAGER)));
        assertEquals(40, BuiltinNavigationProfiles.requiredPathLength(
                new EntityCreature(EntityType.LLAMA)));
        assertEquals(40, BuiltinNavigationProfiles.requiredPathLength(
                new EntityCreature(EntityType.TRADER_LLAMA)));
        assertEquals(32, BuiltinNavigationProfiles.requiredPathLength(
                new EntityCreature(EntityType.FOX)));
        assertEquals(16, BuiltinNavigationProfiles.requiredPathLength(
                new EntityCreature(EntityType.ZOMBIE)));

        NavigationProfile sniffer =
                BuiltinNavigationProfiles.forEntityType(EntityType.SNIFFER);
        assertEquals(-1, sniffer.mobProfile().malus(TerrainType.WATER));
        assertEquals(0, BuiltinNavigationProfiles.withLiveTerrainState(
                sniffer, EntityType.SNIFFER, true, false)
                .mobProfile().malus(TerrainType.WATER));
        assertEquals(0, BuiltinNavigationProfiles.withLiveTerrainState(
                sniffer, EntityType.SNIFFER, false, true)
                .mobProfile().malus(TerrainType.WATER));
        assertSame(sniffer, BuiltinNavigationProfiles.withLiveTerrainState(
                sniffer, EntityType.SNIFFER, false, false));
        assertEquals(-1, sniffer.mobProfile().malus(TerrainType.WATER),
                "request snapshots must not mutate the baseline profile");

        NavigationProfile rider =
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
        NavigationProfile striderRider =
                BuiltinNavigationProfiles.withVehicleTerrainProfile(
                        rider, EntityType.STRIDER);
        assertSame(MobTraversalProfile.STRIDER, striderRider.mobProfile());
        assertEquals(0, striderRider.mobProfile().malus(TerrainType.LAVA));
        assertSame(rider, BuiltinNavigationProfiles.withVehicleTerrainProfile(
                rider, EntityType.HORSE));
    }

    private static void assertProfile(MobTraversalProfile expected, EntityType... types) {
        for (EntityType type : types) {
            assertSame(expected, BuiltinMobProfiles.forEntityType(type), type.key().asString());
        }
    }

    private static void assertOverrides(MobTraversalProfile profile, Object... entries) {
        var expected = new java.util.EnumMap<TerrainType, Double>(TerrainType.class);
        for (int i = 0; i < entries.length; i += 2) {
            expected.put((TerrainType) entries[i], ((Number) entries[i + 1]).doubleValue());
        }
        assertEquals(Map.copyOf(expected), profile.overrides(), profile.name());
    }
}
