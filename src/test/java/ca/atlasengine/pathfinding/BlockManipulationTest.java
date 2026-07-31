package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationModifiers;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.OpenableBlockFamily;
import ca.atlasengine.pathfinding.terrain.TerrainClassifier;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Capability shape, planner transparency, and builder wiring for mobs that
 * change block states.
 */
class BlockManipulationTest {
    /** Sub-cell dimensions so classification covers exactly one block. */
    private static final BoundingBox CELL = new BoundingBox(0.4, 0.4, 0.4);
    private static final Block CLOSED_TRAPDOOR = Block.OAK_TRAPDOOR
            .withProperty("facing", "east")
            .withProperty("half", "bottom")
            .withProperty("open", "false");
    /** Swings along a corridor running down +x rather than across it. */
    private static final Block CROSSABLE_TRAPDOOR = CLOSED_TRAPDOOR
            .withProperty("facing", "north");
    private static final Block CLOSED_GATE = Block.OAK_FENCE_GATE
            .withProperty("facing", "north")
            .withProperty("open", "false");
    private static final Block CLOSED_DOOR = Block.OAK_DOOR
            .withProperty("facing", "east")
            .withProperty("hinge", "left")
            .withProperty("half", "lower")
            .withProperty("open", "false");
    private final TerrainClassifier classifier = new TerrainClassifier();

    @Test
    void capabilityConstantsMatchTheBuiltinDoorContract() {
        assertFalse(BlockManipulationCapabilities.DISABLED.enabled());
        assertTrue(BlockManipulationCapabilities.STANDARD.enabled());
        assertTrue(BlockManipulationCapabilities.STANDARD.manipulates(
                OpenableBlockFamily.DOOR));
        assertFalse(BlockManipulationCapabilities.STANDARD.manipulates(
                OpenableBlockFamily.TRAPDOOR));
        assertFalse(BlockManipulationCapabilities.STANDARD.manipulates(
                OpenableBlockFamily.FENCE_GATE));
        assertFalse(BlockManipulationCapabilities.STANDARD.closesBehind());
        assertEquals(3.0, BlockManipulationCapabilities.STANDARD.closeRange());
        assertThrows(IllegalArgumentException.class,
                () -> BlockManipulationCapabilities.STANDARD.closingBehind(0));
        assertThrows(IllegalArgumentException.class,
                () -> BlockManipulationCapabilities.STANDARD.closingBehind(
                        Double.NaN));
        assertThrows(UnsupportedOperationException.class,
                () -> BlockManipulationCapabilities.STANDARD.families()
                        .add(OpenableBlockFamily.TRAPDOOR));
    }

    @Test
    void builtinProfilesKeepTheirExistingDoorSemantics() {
        assertEquals(BlockManipulationCapabilities.DISABLED,
                MobTraversalProfile.DEFAULT.blockManipulation());
        assertEquals(BlockManipulationCapabilities.STANDARD,
                MobTraversalProfile.VILLAGER.blockManipulation());
        assertEquals(BlockManipulationCapabilities.STANDARD,
                MobTraversalProfile.PIGLIN.blockManipulation());
        assertEquals(BlockManipulationCapabilities.STANDARD,
                MobTraversalProfile.COPPER_GOLEM.blockManipulation());
        assertFalse(MobTraversalProfile.VILLAGER.blockManipulation()
                .closesBehind());
    }

    @Test
    void doorFlagAndFamilySetComposeAndSurviveInheritance() {
        MobTraversalProfile flagLast = MobTraversalProfile.builder("flag_last")
                .blockManipulation(BlockManipulationCapabilities
                        .of(OpenableBlockFamily.values()).closingBehind())
                .canOpenDoors(false)
                .build();
        MobTraversalProfile flagFirst = MobTraversalProfile.builder("flag_first")
                .canOpenDoors(true)
                .blockManipulation(BlockManipulationCapabilities.of(
                        OpenableBlockFamily.TRAPDOOR))
                .build();
        MobTraversalProfile inherited = MobTraversalProfile.builder("inherited")
                .from(flagLast).build();

        assertFalse(flagLast.canOpenDoors());
        assertTrue(flagLast.blockManipulation().manipulates(
                OpenableBlockFamily.TRAPDOOR));
        assertTrue(flagLast.blockManipulation().closesBehind());
        assertFalse(flagFirst.canOpenDoors());
        assertTrue(flagFirst.blockManipulation().manipulates(
                OpenableBlockFamily.TRAPDOOR));
        assertEquals(flagLast.blockManipulation(),
                inherited.blockManipulation());
        assertEquals(BlockManipulationCapabilities.STANDARD,
                MobTraversalProfile.builder("doors").canOpenDoors(true)
                        .build().blockManipulation());
    }

    @Test
    void requestModifierGrantsManipulationWithoutMutatingTheSharedProfile() {
        NavigationProfile base = BuiltinNavigationProfiles.forEntityType(
                EntityType.ZOMBIE);
        NavigationModifiers modifier = NavigationModifiers.builder()
                .blockManipulation(BlockManipulationCapabilities.of(
                                OpenableBlockFamily.DOOR,
                                OpenableBlockFamily.FENCE_GATE)
                        .closingBehind())
                .build();

        NavigationProfile scoped = modifier.applyTo(base);

        assertTrue(scoped.mobProfile().blockManipulation().manipulates(
                OpenableBlockFamily.FENCE_GATE));
        assertTrue(scoped.mobProfile().blockManipulation().closesBehind());
        assertTrue(scoped.mobProfile().canOpenDoors());
        assertEquals(BlockManipulationCapabilities.DISABLED,
                base.mobProfile().blockManipulation());
        assertEquals(BlockManipulationCapabilities.DISABLED,
                MobTraversalProfile.DEFAULT.blockManipulation());
        assertSame(base, NavigationModifiers.NONE.applyTo(base));
    }

    @Test
    void classifierWidensOnlyTheConfiguredFamilies() {
        TestWorld world = new TestWorld()
                .set(0, 0, 0, Block.STONE).set(0, 1, 0, CLOSED_TRAPDOOR)
                .set(1, 0, 0, Block.STONE).set(1, 1, 0, CLOSED_GATE)
                .set(2, 0, 0, Block.STONE).set(2, 1, 0, Block.LILY_PAD)
                .set(3, 0, 0, Block.STONE).set(3, 1, 0, Block.OAK_FENCE)
                .set(4, 0, 0, Block.STONE).set(4, 1, 0, Block.IRON_TRAPDOOR);

        MobTraversalProfile plain = MobTraversalProfile.DEFAULT;
        MobTraversalProfile manipulating = MobTraversalProfile
                .builder("manipulating")
                .blockManipulation(BlockManipulationCapabilities.of(
                        OpenableBlockFamily.values()))
                .build();

        assertEquals(TerrainType.TRAPDOOR, classify(world, 0, plain));
        assertEquals(TerrainType.FENCE, classify(world, 1, plain));
        assertEquals(TerrainType.WALKABLE_DOOR, classify(world, 0, manipulating));
        assertEquals(TerrainType.WALKABLE_DOOR, classify(world, 1, manipulating));
        assertEquals(TerrainType.TRAPDOOR, classify(world, 2, manipulating),
                "lily pads share the TRAPDOOR path type but not the family");
        assertEquals(TerrainType.FENCE, classify(world, 3, manipulating),
                "an ordinary fence is not a fence gate");
        assertEquals(TerrainType.TRAPDOOR, classify(world, 4, manipulating),
                "iron trapdoors cannot be opened by hand");
    }

    /**
     * The modern evaluator prices a closed trapdoor at zero, so the refusal
     * being lifted here has to come from a profile that rejects one.
     */
    @Test
    void groundSearchStopsRejectingManipulableTrapdoorsAndGates() {
        TestWorld world = new TestWorld()
                .floor(-1, 3, 0, 0, 0, Block.STONE)
                .set(1, 1, 0, CROSSABLE_TRAPDOOR)
                .set(2, 1, 0, CLOSED_GATE);
        MobTraversalProfile plain = MobTraversalProfile.builder("plain")
                .malus(TerrainType.TRAPDOOR, -1)
                .build();
        MobTraversalProfile manipulating = MobTraversalProfile
                .builder("manipulating")
                .from(plain)
                .blockManipulation(BlockManipulationCapabilities.of(
                        OpenableBlockFamily.TRAPDOOR,
                        OpenableBlockFamily.FENCE_GATE))
                .build();

        PathResult opened = cross(world, manipulating);

        assertFalse(cross(world, plain).found());
        assertTrue(opened.found(), opened::toString);
    }

    @Test
    void collisionViewHidesOnlyClosedBlocksOfConfiguredFamilies() {
        TestWorld world = new TestWorld()
                .set(0, 1, 0, CLOSED_DOOR)
                .set(1, 1, 0, CLOSED_TRAPDOOR)
                .set(2, 1, 0, CLOSED_GATE)
                .set(3, 1, 0, CLOSED_DOOR.withProperty("open", "true"))
                .set(4, 1, 0, Block.IRON_DOOR.withProperty("open", "false"));

        Block.Getter doorsOnly =
                MobTraversalProfile.VILLAGER.collisionView(world);
        Block.Getter everything = MobTraversalProfile.builder("everything")
                .blockManipulation(BlockManipulationCapabilities.of(
                        OpenableBlockFamily.values()))
                .build().collisionView(world);

        assertTrue(block(doorsOnly, 0).air());
        assertFalse(block(doorsOnly, 1).air());
        assertFalse(block(doorsOnly, 2).air());
        assertTrue(block(everything, 0).air());
        assertTrue(block(everything, 1).air());
        assertTrue(block(everything, 2).air());
        assertFalse(block(everything, 3).air(),
                "an already open door keeps its own collision shape");
        assertFalse(block(everything, 4).air());
        assertSame(world, MobTraversalProfile.DEFAULT.collisionView(world));
        assertSame(world, MobTraversalProfile.builder("shut")
                .from(MobTraversalProfile.VILLAGER)
                .canPassDoors(false)
                .build().collisionView(world));
    }

    @Test
    void openableFamilyMatchesTheHandOpenableBlockSetTypes() {
        assertEquals(OpenableBlockFamily.DOOR,
                BlockTraversalData.openableFamily(Block.OAK_DOOR));
        assertEquals(OpenableBlockFamily.DOOR,
                BlockTraversalData.openableFamily(Block.COPPER_DOOR));
        assertEquals(OpenableBlockFamily.TRAPDOOR,
                BlockTraversalData.openableFamily(Block.OAK_TRAPDOOR));
        assertEquals(OpenableBlockFamily.TRAPDOOR,
                BlockTraversalData.openableFamily(Block.COPPER_TRAPDOOR));
        assertEquals(OpenableBlockFamily.FENCE_GATE,
                BlockTraversalData.openableFamily(Block.OAK_FENCE_GATE));
        assertNull(BlockTraversalData.openableFamily(Block.IRON_DOOR));
        assertNull(BlockTraversalData.openableFamily(Block.IRON_TRAPDOOR));
        assertNull(BlockTraversalData.openableFamily(Block.OAK_FENCE));
        assertNull(BlockTraversalData.openableFamily(Block.LILY_PAD));
    }

    private static PathResult cross(TestWorld world,
                                    MobTraversalProfile profile) {
        return new DiscreteGroundPathfinder().findPath(world, new Vec(-0.5, 1, 0.5), new Vec(3.5, 1, 0.5), CELL, profile, GroundSearchLimits.builder().maxPathLength(16).reachRange(0).maxVisitedMultiplier(4).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
    }

    private TerrainType classify(TestWorld world, int x,
                                 MobTraversalProfile profile) {
        return classifier.classifyAnchored(world, x, 1, 0, CELL, profile);
    }

    private static Block block(Block.Getter blocks, int x) {
        return blocks.getBlock(x, 1, 0, Block.Getter.Condition.TYPE);
    }
}
