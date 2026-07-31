package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.terrain.BlockTagIndex;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.TerrainClassifier;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BlockTraversalDataTest {
    private final TerrainClassifier classifier = new TerrainClassifier();

    @Test
    void registryTagsResolveDirectAndNestedMembers() {
        assertTrue(BlockTagIndex.contains("wooden_fences", Block.OAK_FENCE));
        assertTrue(BlockTagIndex.contains("fences", Block.OAK_FENCE),
                "fences includes the nested wooden_fences tag");
        assertTrue(BlockTagIndex.contains("minecraft:walls", Block.SULFUR_WALL));
        assertFalse(BlockTagIndex.contains("fences", Block.STONE));
        assertFalse(BlockTagIndex.contains("no_such_tag", Block.OAK_FENCE),
                "an unknown tag is empty rather than an error");
    }

    @Test
    void climbableTagContainsExpectedBuiltInBlocks() {
        for (Block climbable : List.of(
                Block.LADDER, Block.VINE, Block.SCAFFOLDING,
                Block.WEEPING_VINES, Block.WEEPING_VINES_PLANT,
                Block.TWISTING_VINES, Block.TWISTING_VINES_PLANT,
                Block.CAVE_VINES, Block.CAVE_VINES_PLANT)) {
            assertTrue(BlockTagIndex.contains("climbable", climbable),
                    () -> climbable.key() + " is climbable in 26.2");
        }
        assertFalse(BlockTagIndex.contains("climbable", Block.STONE));
        assertFalse(BlockTagIndex.contains("climbable", Block.OAK_FENCE));
    }

    @Test
    void structuralPathFamiliesUseBundledTagMembership() {
        assertTrue(BlockTraversalData.isTrapdoor(Block.OXIDIZED_COPPER_TRAPDOOR));
        assertTrue(BlockTraversalData.isRail(Block.ACTIVATOR_RAIL));
        assertTrue(BlockTraversalData.isLeaves(Block.FLOWERING_AZALEA_LEAVES));
        assertTrue(BlockTraversalData.isFence(Block.NETHER_BRICK_FENCE));
        assertTrue(BlockTraversalData.isFence(Block.CINNABAR_BRICK_WALL));
        assertTrue(BlockTraversalData.isSpeleothem(Block.SULFUR_SPIKE));
        assertTrue(BlockTraversalData.isClimbable(Block.LADDER));
        assertTrue(BlockTraversalData.isClimbable(Block.SCAFFOLDING));
        assertTrue(BlockTraversalData.isClimbable(Block.CAVE_VINES));
    }

    @Test
    void onlyOpenFacingAlignedTrapdoorAboveLadderIsClimbable() {
        TestWorld world = new TestWorld().set(0, 0, 0,
                Block.LADDER.withProperty("facing", "north"));
        Block aligned = Block.OAK_TRAPDOOR
                .withProperty("facing", "north")
                .withProperty("open", "true");
        world.set(0, 1, 0, aligned);
        assertTrue(BlockTraversalData.isClimbableAt(world, 0, 1, 0));

        world.set(0, 1, 0, aligned.withProperty("open", "false"));
        assertFalse(BlockTraversalData.isClimbableAt(world, 0, 1, 0));
        world.set(0, 1, 0, aligned.withProperty("facing", "south"));
        assertFalse(BlockTraversalData.isClimbableAt(world, 0, 1, 0));
    }

    @Test
    void copperDoorsUseHandOpenableDoorSemantics() {
        assertTrue(BlockTraversalData.isHandOpenableDoor(Block.COPPER_DOOR));
        assertTrue(BlockTraversalData.isHandOpenableDoor(Block.OAK_DOOR));
        assertFalse(BlockTraversalData.isHandOpenableDoor(Block.IRON_DOOR));
        assertEquals(TerrainType.DOOR_WOOD_CLOSED,
                classifier.raw(Block.COPPER_DOOR.withProperty("open", "false")));
        assertEquals(TerrainType.DOOR_IRON_CLOSED,
                classifier.raw(Block.IRON_DOOR.withProperty("open", "false")));
    }

    @Test
    void landPathfindabilityUsesCollisionShapeAndBlockOverrides() {
        assertTrue(BlockTraversalData.isLandPathfindable(Block.WHITE_CARPET));
        assertTrue(BlockTraversalData.isLandPathfindable(Block.CANDLE));
        assertFalse(BlockTraversalData.isLandPathfindable(Block.STONE));
        assertFalse(BlockTraversalData.isLandPathfindable(Block.OAK_SLAB));
        assertFalse(BlockTraversalData.isLandPathfindable(Block.OAK_STAIRS));
        assertFalse(BlockTraversalData.isLandPathfindable(Block.CAKE));
        assertFalse(BlockTraversalData.isLandPathfindable(Block.IRON_BARS));
        assertFalse(BlockTraversalData.isLandPathfindable(Block.GLASS_PANE));
        assertFalse(BlockTraversalData.isLandPathfindable(Block.CREEPER_HEAD));
        assertFalse(BlockTraversalData.isLandPathfindable(Block.DRIED_GHAST));
        assertFalse(BlockTraversalData.isLandPathfindable(Block.SNIFFER_EGG));
        assertFalse(BlockTraversalData.isLandPathfindable(Block.PISTON_HEAD));
        assertFalse(BlockTraversalData.isLandPathfindable(Block.END_ROD));
        assertTrue(BlockTraversalData.isLandPathfindable(
                Block.SNOW.withProperty("layers", "4")));
        assertFalse(BlockTraversalData.isLandPathfindable(
                Block.SNOW.withProperty("layers", "5")));
    }

    @Test
    void waterloggedSlabsPermitSwimmingButStairsDoNot() {
        Block slab = Block.OAK_SLAB.withProperty("waterlogged", "true");
        Block stair = Block.OAK_STAIRS.withProperty("waterlogged", "true");

        assertTrue(BlockTraversalData.hasWaterFluid(slab));
        assertTrue(BlockTraversalData.hasWaterFluid(stair));
        assertTrue(BlockTraversalData.isWaterPathfindable(slab));
        assertFalse(BlockTraversalData.isWaterPathfindable(stair));
    }

    @Test
    void burningClassificationHonorsCampfireLitState() {
        assertTrue(BlockTraversalData.isBurning(Block.CAMPFIRE.withProperty("lit", "true")));
        assertFalse(BlockTraversalData.isBurning(Block.CAMPFIRE.withProperty("lit", "false")));
        assertTrue(BlockTraversalData.isBurning(Block.SOUL_FIRE));
        assertTrue(BlockTraversalData.isBurning(Block.LAVA_CAULDRON));
    }

    @Test
    void classifierTreatsPartialCollisionOverridesAsExpected() {
        assertEquals(TerrainType.OPEN, classifier.raw(Block.WHITE_CARPET));
        assertEquals(TerrainType.OPEN, classifier.raw(Block.CANDLE));
        assertEquals(TerrainType.BLOCKED, classifier.raw(Block.END_ROD));
        assertEquals(TerrainType.BLOCKED, classifier.raw(Block.OAK_SLAB));
        assertEquals(TerrainType.BLOCKED, classifier.raw(
                Block.OAK_SLAB.withProperty("waterlogged", "true")));
        assertEquals(TerrainType.BLOCKED, classifier.raw(Block.CAKE));
        assertEquals(TerrainType.FENCE, classifier.raw(Block.CINNABAR_WALL));
    }
}
