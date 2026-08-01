package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.BuiltinMobProfiles;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainClassifier;
import ca.atlasengine.pathfinding.terrain.TerrainCosts;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class TerrainPolicyTest {
    private static final BoundingBox MOB = new BoundingBox(0.6, 1.8, 0.6);
    private final TerrainClassifier classifier = new TerrainClassifier();

    /** Pins the stable defaults used when a profile supplies no override. */
    @Test
    void baselineCostsMatchTargetedGameVersion() {
        assertEquals(-1, TerrainCosts.baseline(TerrainType.BLOCKED));
        assertEquals(0, TerrainCosts.baseline(TerrainType.WALKABLE));
        assertEquals(-1, TerrainCosts.baseline(TerrainType.LAVA));
        assertEquals(8, TerrainCosts.baseline(TerrainType.WATER));
        assertEquals(0, TerrainCosts.baseline(TerrainType.WATER_BORDER));
        assertEquals(8, TerrainCosts.baseline(TerrainType.FIRE_IN_NEIGHBOR));
        assertEquals(16, TerrainCosts.baseline(TerrainType.FIRE));
        assertEquals(-1, TerrainCosts.baseline(TerrainType.DAMAGING));
        assertEquals(4, TerrainCosts.baseline(TerrainType.BREACH));
        assertEquals(8, TerrainCosts.baseline(TerrainType.STICKY_HONEY));
        assertEquals(4,
                TerrainCosts.baseline(TerrainType.BIG_MOBS_CLOSE_TO_DANGER));
        assertEquals(27, TerrainType.values().length);
    }

    /**
     * Every category must be priced, and the deprecated accessor must stay a
     * faithful alias for the table it now delegates to.
     */
    @Test
    @SuppressWarnings("deprecation")
    void everyCategoryHasABaselineCost() {
        for (TerrainType type : TerrainType.values()) {
            assertDoesNotThrow(() -> TerrainCosts.baseline(type), type.name());
            assertEquals(TerrainCosts.baseline(type), type.defaultMalus(),
                    type.name());
        }
    }

    /** A profile override must win over the baseline for that category only. */
    @Test
    void profileOverridesLayerOverBaseline() {
        MobTraversalProfile lavaWalker = MobTraversalProfile.builder("test")
                .malus(TerrainType.LAVA, 0)
                .build();

        assertEquals(0, lavaWalker.malus(TerrainType.LAVA));
        assertEquals(TerrainCosts.baseline(TerrainType.WATER),
                lavaWalker.malus(TerrainType.WATER));
    }

    /**
     * TerrainType is exported, so ordinals are public API: they drive consumer
     * EnumMap iteration order, ordinal-indexed arrays, and numeric
     * serialization. Reordering breaks those silently rather than loudly, so
     * this pins the whole sequence. Append new categories at the end only.
     */
    @Test
    void declarationOrderIsStableBecauseOrdinalsAreExported() {
        String[] expected = {
                "BLOCKED", "OPEN", "WALKABLE", "WALKABLE_DOOR", "TRAPDOOR",
                "POWDER_SNOW", "ON_TOP_OF_POWDER_SNOW", "FENCE", "LAVA",
                "WATER", "WATER_BORDER", "RAIL", "UNPASSABLE_RAIL",
                "FIRE_IN_NEIGHBOR", "FIRE", "DAMAGING_IN_NEIGHBOR", "DAMAGING",
                "DOOR_OPEN", "DOOR_WOOD_CLOSED", "DOOR_IRON_CLOSED", "BREACH",
                "LEAVES", "STICKY_HONEY", "COCOA", "DAMAGE_CAUTIOUS",
                "ON_TOP_OF_TRAPDOOR", "BIG_MOBS_CLOSE_TO_DANGER"
        };

        assertArrayEquals(expected,
                java.util.Arrays.stream(TerrainType.values())
                        .map(Enum::name).toArray(String[]::new));
    }

    @Test
    void rawClassifierMatchesExpectedHazardFamilies() {
        assertEquals(TerrainType.DAMAGING, classifier.raw(Block.CACTUS));
        assertEquals(TerrainType.DAMAGING, classifier.raw(Block.SWEET_BERRY_BUSH));
        assertEquals(TerrainType.FIRE, classifier.raw(Block.FIRE));
        assertEquals(TerrainType.FIRE, classifier.raw(Block.MAGMA_BLOCK));
        assertEquals(TerrainType.LAVA, classifier.raw(Block.LAVA));
        assertEquals(TerrainType.WATER, classifier.raw(Block.WATER));
        assertEquals(TerrainType.STICKY_HONEY, classifier.raw(Block.HONEY_BLOCK));
        assertEquals(TerrainType.DAMAGE_CAUTIOUS, classifier.raw(Block.WITHER_ROSE));
        assertEquals(TerrainType.POWDER_SNOW, classifier.raw(Block.POWDER_SNOW));
        assertEquals(TerrainType.RAIL, classifier.raw(Block.RAIL));
        assertEquals(TerrainType.FENCE, classifier.raw(Block.OAK_FENCE));
        assertEquals(TerrainType.DAMAGE_CAUTIOUS, classifier.raw(Block.SULFUR_SPIKE));
    }

    @Test
    void railAndFenceBelowOpenNodeBecomeWalkableRatherThanTheirRawType() {
        TestWorld rail = new TestWorld().set(0, 0, 0, Block.RAIL);
        TestWorld fence = new TestWorld().set(0, 0, 0, Block.OAK_FENCE);

        assertEquals(TerrainType.WALKABLE,
                classifier.classify(rail, new Pos(0.5, 1, 0.5), MOB,
                        MobTraversalProfile.DEFAULT));
        assertEquals(TerrainType.WALKABLE,
                classifier.classify(fence, new Pos(0.5, 1, 0.5), MOB,
                        MobTraversalProfile.DEFAULT));
    }

    @Test
    void entityTypeRegistrySelectsSpecialProfiles() {
        assertSame(MobTraversalProfile.STRIDER, BuiltinMobProfiles.forEntityType(EntityType.STRIDER));
        assertSame(MobTraversalProfile.ENDERMAN, BuiltinMobProfiles.forEntityType(EntityType.ENDERMAN));
        assertSame(MobTraversalProfile.VILLAGER, BuiltinMobProfiles.forEntityType(EntityType.VILLAGER));
        assertSame(MobTraversalProfile.FOX, BuiltinMobProfiles.forEntityType(EntityType.FOX));
        assertSame(MobTraversalProfile.WATER_ANIMAL, BuiltinMobProfiles.forEntityType(EntityType.COD));
        assertSame(MobTraversalProfile.BEE, BuiltinMobProfiles.forEntityType(EntityType.BEE));
        assertSame(MobTraversalProfile.SNIFFER, BuiltinMobProfiles.forEntityType(EntityType.SNIFFER));
        assertSame(MobTraversalProfile.CAMEL, BuiltinMobProfiles.forEntityType(EntityType.CAMEL));
        assertSame(MobTraversalProfile.BREEZE, BuiltinMobProfiles.forEntityType(EntityType.BREEZE));
        assertSame(MobTraversalProfile.COPPER_GOLEM,
                BuiltinMobProfiles.forEntityType(EntityType.COPPER_GOLEM));
    }

    @Test
    void specializedMobCostsMatchExpectedProfiles() {
        assertEquals(-1, MobTraversalProfile.BEE.malus(TerrainType.WATER));
        assertEquals(16, MobTraversalProfile.BEE.malus(TerrainType.WATER_BORDER));
        assertEquals(-1, MobTraversalProfile.PARROT.malus(TerrainType.COCOA));
        assertEquals(-1, MobTraversalProfile.SNIFFER.malus(TerrainType.DAMAGE_CAUTIOUS));
        assertEquals(-1, MobTraversalProfile.GOAT.malus(TerrainType.POWDER_SNOW));
        assertTrue(MobTraversalProfile.CAMEL.canWalkOverFences());
        assertEquals(0, MobTraversalProfile.RAVAGER.malus(TerrainType.LEAVES));
        assertEquals(8, MobTraversalProfile.WITHER_SKELETON.malus(TerrainType.LAVA));
        assertEquals(-1, MobTraversalProfile.BREEZE.malus(TerrainType.FIRE));
        assertTrue(MobTraversalProfile.COPPER_GOLEM.canOpenDoors());
        assertEquals(-1, MobTraversalProfile.TURTLE.malus(TerrainType.DOOR_OPEN));
    }

    @Test
    void customProfileOverridesAreImmutableAndModular() {
        MobTraversalProfile custom = MobTraversalProfile.builder("fireproof_cow")
                .from(MobTraversalProfile.ANIMAL)
                .malus(TerrainType.FIRE, 0)
                .malus(TerrainType.FIRE_IN_NEIGHBOR, 0)
                .build();
        assertEquals(0, custom.malus(TerrainType.FIRE));
        assertEquals(-1, MobTraversalProfile.ANIMAL.malus(TerrainType.FIRE));
        assertThrows(UnsupportedOperationException.class,
                () -> custom.overrides().put(TerrainType.LAVA, 0.0));
    }

    @Test
    void camelUsesItsStepHeightToCrossFence() {
        TestWorld world = lane();
        for (int x = -2; x <= 8; x++) {
            world.column(x, -1, 1, 4, Block.STONE);
            world.column(x, 1, 1, 4, Block.STONE);
        }
        world.set(2, 1, 0, Block.OAK_FENCE);
        BoundingBox box = new BoundingBox(0.9, 1.8, 0.9);
        EntityPathfinder pathfinder = new EntityPathfinder();

        PathResult camel = pathfinder.findPath(NavigationRequest.builder(
                        world, new Pos(0.5, 1, 0.5), new Pos(5.5, 1, 0.5), box,
                        BuiltinNavigationProfiles.forEntityType(
                                EntityType.CAMEL))
                .maxPathLength(20)
                .maxVisitedMultiplier(8)
                .build(), SearchControl.NONE);
        PathResult cow = pathfinder.findPath(NavigationRequest.builder(
                        world, new Pos(0.5, 1, 0.5), new Pos(5.5, 1, 0.5), box,
                        BuiltinNavigationProfiles.forEntityType(EntityType.COW))
                .maxPathLength(20)
                .maxVisitedMultiplier(8)
                .build(), SearchControl.NONE);
        NavigationProfile tallButForbidden = NavigationProfile.builder(NavigationMode.GROUND, MobTraversalProfile.DEFAULT, GroundCapabilities.builder().maxStepHeight(1.5).maxFallDistance(5).allowDiagonal(true).build()).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();
        PathResult noFenceCapability = pathfinder.findPath(
                NavigationRequest.builder(
                                world, new Pos(0.5, 1, 0.5),
                                new Pos(5.5, 1, 0.5), box, tallButForbidden)
                        .maxPathLength(20)
                        .maxVisitedMultiplier(8)
                        .build(), SearchControl.NONE);

        assertTrue(camel.found(), camel::toString);
        assertTrue(camel.nodes().stream().anyMatch(node -> node.y() > 2),
                "camel must actually step over the 1.5-block collision shape");
        assertFalse(cow.found(), "ordinary one-block step height cannot cross a fence");
        assertFalse(noFenceCapability.found(),
                "step height alone must not bypass canWalkOverFences");
    }

    @Test
    void platformJumpCapabilityCannotBypassFencePermission() {
        TestWorld world = lane();
        for (int x = -2; x <= 8; x++) {
            world.column(x, -1, 1, 4, Block.STONE);
            world.column(x, 1, 1, 4, Block.STONE);
        }
        world.set(2, 1, 0, Block.OAK_FENCE);
        GroundCapabilities jumping = GroundCapabilities.builder().maxStepHeight(1).maxFallDistance(5).allowDiagonal(true).build()
                .withPlatformJump(PlatformJumpCapabilities.builder().maxHorizontalDistance(4).maxRise(1).maxDrop(2).apexClearance(1).build());
        NavigationProfile profile = NavigationProfile.builder(NavigationMode.GROUND, MobTraversalProfile.DEFAULT, jumping).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();

        PathResult result = new EntityPathfinder().findPath(
                NavigationRequest.builder(world,
                                new Pos(0.5, 1, 0.5),
                                new Pos(5.5, 1, 0.5),
                                new BoundingBox(0.8, 1.8, 0.8), profile)
                        .maxPathLength(20)
                        .maxVisitedMultiplier(8)
                        .build(),
                SearchControl.NONE);

        assertFalse(result.found(), result::toString);
        assertTrue(result.nodes().stream().noneMatch(node ->
                        node.movement() == PathNode.Movement.JUMP),
                "a fence is an obstacle, not an unsupported platform gap");
    }

    @Test
    void discreteHorizontalVolumeUsesIndependentWidthAndDepth() {
        TestWorld world = new TestWorld().floor(
                -2, 4, -2, 4, 0, Block.STONE);
        world.set(0, 1, 1, Block.STONE);
        BoundingBox nonSquare = new BoundingBox(1.8, 1.8, 0.2);

        TerrainType type = classifier.classifyAnchored(
                world, 0, 1, 0, nonSquare, MobTraversalProfile.DEFAULT);

        assertEquals(TerrainType.WALKABLE, type,
                "a custom shallow Z footprint must not include the adjacent "
                        + "row merely because the entity is wide on X");
    }

    @Test
    void wardenUsesHorizontalOnlyMovementEdgeCost() {
        assertTrue(BuiltinNavigationProfiles.forEntityType(EntityType.WARDEN)
                .groundCapabilities().horizontalEdgeCost());
        assertFalse(BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE)
                .groundCapabilities().horizontalEdgeCost());
    }

    @Test
    void amphibiousWaterBorderIsClassifiedBeforeVolumeSelection() {
        TestWorld world = new TestWorld()
                .floor(-2, 4, -2, 4, 0, Block.STONE)
                .set(0, 1, 0, Block.WATER);
        BoundingBox wide = new BoundingBox(1.01, 0.5, 1.01);
        MobTraversalProfile profile = MobTraversalProfile.builder("shoreline")
                .malus(TerrainType.WATER, 0)
                .malus(TerrainType.WATER_BORDER, 4)
                .build();

        TerrainType type = classifier.classifyAnchoredAmphibious(
                world, 0, 1, 0, wide, profile);

        assertEquals(TerrainType.WATER_BORDER, type,
                "each WATER cell becomes WATER_BORDER before the complete "
                        + "entity volume is aggregated");
    }

    private static TestWorld lane() {
        return new TestWorld().floor(-2, 8, -3, 3, 0, Block.STONE);
    }

}
