package ca.atlasengine.pathfinding.terrain;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block;
import net.minestom.server.instance.block.BlockFace;

import java.util.Set;
import java.util.BitSet;

/**
 * Block-family predicates used by the navigation terrain classifiers.
 *
 * <p>Structural families come from Minestom's block tags. Land
 * pathfindability is based on collision shape, with explicit handling for
 * partial-collision block families.</p>
 */
public final class BlockTraversalData {
    private static final BlockFace[] BLOCK_FACES = BlockFace.values();
    private static final Set<String> LAND_NEVER_IDS = Set.of(
            "bamboo", "cake", "sea_pickle", "end_portal_frame",
            "lightning_rod", "end_rod", "hopper", "bell", "lantern", "dirt_path",
            "cocoa", "grindstone", "enchanting_table", "respawn_anchor",
            "decorated_pot", "sculk_sensor", "stonecutter", "soul_sand",
            "azalea", "flowering_azalea", "farmland", "heavy_core",
            "chorus_plant", "chest", "trapped_chest", "ender_chest",
            "brewing_stand", "conduit", "lectern", "iron_chain", "copper_chain", "exposed_copper_chain",
            "weathered_copper_chain", "oxidized_copper_chain",
            "waxed_copper_chain", "waxed_exposed_copper_chain",
            "waxed_weathered_copper_chain", "waxed_oxidized_copper_chain", "composter",
            "dragon_egg", "mud", "acacia_shelf", "bamboo_shelf", "birch_shelf", "cherry_shelf",
            "crimson_shelf", "dark_oak_shelf", "jungle_shelf",
            "mangrove_shelf", "oak_shelf", "pale_oak_shelf",
            "spruce_shelf", "warped_shelf", "dried_ghast", "sniffer_egg",
            "piston", "sticky_piston", "piston_head", "moving_piston",
            "glass_pane", "white_stained_glass_pane", "orange_stained_glass_pane",
            "magenta_stained_glass_pane", "light_blue_stained_glass_pane",
            "yellow_stained_glass_pane", "lime_stained_glass_pane",
            "pink_stained_glass_pane", "gray_stained_glass_pane",
            "light_gray_stained_glass_pane", "cyan_stained_glass_pane",
            "purple_stained_glass_pane", "blue_stained_glass_pane",
            "brown_stained_glass_pane", "green_stained_glass_pane",
            "red_stained_glass_pane", "black_stained_glass_pane",
            "skeleton_skull", "skeleton_wall_skull", "wither_skeleton_skull",
            "wither_skeleton_wall_skull", "zombie_head", "zombie_wall_head",
            "player_head", "player_wall_head", "creeper_head", "creeper_wall_head",
            "dragon_head", "dragon_wall_head", "piglin_head", "piglin_wall_head"
    );
    private static final String[] LAND_NEVER_TAGS = {
            "slabs", "stairs", "beds", "anvil", "campfires", "candle_cakes",
            "cauldrons", "flower_pots", "wall_hanging_signs", "bars",
            "copper_golem_statues"
    };
    private static final BitSet TRAPDOORS = tag("trapdoors");
    private static final BitSet DOORS = tag("doors");
    private static final BitSet RAILS = tag("rails");
    private static final BitSet LEAVES = tag("leaves");
    private static final BitSet FENCES = tag("fences");
    private static final BitSet WALLS = tag("walls");
    private static final BitSet FENCE_GATES = tag("fence_gates");
    private static final BitSet SPELEOTHEMS = tag("speleothems");
    private static final BitSet CLIMBABLE = tag("climbable");
    private static final BitSet FIRE = tag("fire");
    private static final BitSet CAMPFIRES = tag("campfires");
    private static final BitSet BARS = tag("bars");
    private static final BitSet STAIRS = tag("stairs");
    private static final BitSet SLABS = tag("slabs");
    private static final BitSet LAND_NEVER_BLOCKS = ids(LAND_NEVER_IDS);
    private static final BitSet[] LAND_NEVER_MEMBERS =
            java.util.Arrays.stream(LAND_NEVER_TAGS)
                    .map(BlockTraversalData::tag).toArray(BitSet[]::new);

    private BlockTraversalData() {
    }

    public static boolean isTrapdoor(Block block) {
        return contains(TRAPDOORS, block);
    }

    public static boolean isDoor(Block block) {
        return contains(DOORS, block);
    }

    /**
     * Copper doors are hand-openable too; the wooden-door tag alone is
     * therefore insufficient.
     */
    public static boolean isHandOpenableDoor(Block block) {
        return isDoor(block) && !block.compare(Block.IRON_DOOR);
    }

    /**
     * Only the iron trapdoor refuses hand opening.
     */
    public static boolean isHandOpenableTrapdoor(Block block) {
        return isTrapdoor(block) && !block.compare(Block.IRON_TRAPDOOR);
    }

    /**
     * The planner routes through these as if open; the follower opens them on
     * arrival. Both sides must agree, so both read this one predicate.
     */
    public static boolean isClosedHandOpenableDoor(Block block) {
        return isHandOpenableDoor(block) && !isOpen(block);
    }

    public static boolean isOpen(Block block) {
        return Boolean.parseBoolean(property(block, "open", "false"));
    }

    /**
     * The family a hand could toggle, or {@code null} for everything else.
     * Fence gates have no block set type and are always hand-openable.
     */
    public static OpenableBlockFamily openableFamily(Block block) {
        if (isHandOpenableDoor(block)) return OpenableBlockFamily.DOOR;
        if (isHandOpenableTrapdoor(block)) return OpenableBlockFamily.TRAPDOOR;
        if (isFenceGate(block)) return OpenableBlockFamily.FENCE_GATE;
        return null;
    }

    /**
     * The state a manipulated openable stands in while the mob traverses it:
     * a closed one is about to be swung by the follower, an open one already
     * has been. Returns the block itself for every other family, so callers
     * that ignore the distinction keep reading the world.
     */
    public static Block afterOpening(
            Block block, BlockManipulationCapabilities capabilities) {
        if (!capabilities.manipulates(openableFamily(block))
                || isOpen(block)) {
            return block;
        }
        return block.withProperty("open", "true");
    }

    /**
     * Whether the panel an openable swings to would stand inside an entity
     * box placed at {@code relativePosition}, measured from this block's own
     * corner. An open trapdoor or door is a three-sixteenths panel across one
     * face, not air, so a profile that opens one has to fit past it; a fence
     * gate opens to nothing and never obstructs.
     *
     * <p>Only families {@code capabilities} names are considered, and callers
     * must already know the profile can pass doors, since a profile that
     * cannot never has one opened for it.</p>
     */
    public static boolean obstructsWhenOpen(
            Block block, BlockManipulationCapabilities capabilities,
            Point relativePosition, BoundingBox box) {
        if (!capabilities.manipulates(openableFamily(block))) return false;
        return afterOpening(block, capabilities).collisionShape()
                .intersectBox(relativePosition, box);
    }

    /** Extends {@link #isClosedHandOpenableDoor} to the opt-in families. */
    public static boolean isClosedManipulable(
            Block block, BlockManipulationCapabilities capabilities) {
        OpenableBlockFamily family = openableFamily(block);
        if (!capabilities.manipulates(family)) return false;
        return family == OpenableBlockFamily.DOOR
                ? isClosedHandOpenableDoor(block) : !isOpen(block);
    }

    public static boolean isRail(Block block) {
        return contains(RAILS, block);
    }

    public static boolean isLeaves(Block block) {
        return contains(LEAVES, block);
    }

    public static boolean isFence(Block block) {
        return contains(FENCES, block) || contains(WALLS, block);
    }

    public static boolean isFenceGate(Block block) {
        return contains(FENCE_GATES, block);
    }

    public static boolean isSpeleothem(Block block) {
        return contains(SPELEOTHEMS, block);
    }

    /** Membership in the runtime {@code climbable} block tag. */
    public static boolean isClimbable(Block block) {
        return contains(CLIMBABLE, block);
    }

    /**
     * LivingEntity also treats an aligned open trapdoor immediately above a
     * ladder as climbable, although the trapdoor itself is not in the tag.
     */
    public static boolean isClimbableAt(
            Block.Getter blocks, int x, int y, int z) {
        Block block = blocks.getBlock(x, y, z, Block.Getter.Condition.TYPE);
        if (isClimbable(block)) return true;
        if (!isTrapdoor(block)
                || !Boolean.parseBoolean(
                property(block, "open", "false"))) {
            return false;
        }
        Block below = blocks.getBlock(
                x, y - 1, z, Block.Getter.Condition.TYPE);
        return below.compare(Block.LADDER)
                && below.getProperty("facing") != null
                && below.getProperty("facing").equals(
                block.getProperty("facing"));
    }

    public static boolean isLandPathfindable(Block block) {
        if (contains(LAND_NEVER_BLOCKS, block)) return false;
        for (BitSet tag : LAND_NEVER_MEMBERS) {
            if (contains(tag, block)) return false;
        }
        // Thin snow remains traversable below five layers.
        if (block.compare(Block.SNOW)) {
            return Integer.parseInt(property(block, "layers", "1")) < 5;
        }
        // The default BlockBehaviour rule is the negation of a full collision
        // cube. A full cube necessarily exposes all six full collision faces.
        for (BlockFace face : BLOCK_FACES) {
            if (!block.collisionShape().isFaceFull(face)) return true;
        }
        return false;
    }

    public static boolean isBurning(Block block) {
        if (contains(FIRE, block)
                || block.compare(Block.LAVA)
                || block.compare(Block.MAGMA_BLOCK)
                || block.compare(Block.LAVA_CAULDRON)) {
            return true;
        }
        return contains(CAMPFIRES, block)
                && Boolean.parseBoolean(property(block, "lit", "true"));
    }

    public static boolean hasWaterFluid(Block block) {
        return block.compare(Block.WATER)
                || Boolean.parseBoolean(
                        property(block, "waterlogged", "false"));
    }

    /**
     * WATER pathfindability after fluid membership. Most waterlogged blocks
     * inherit the fluid-based default; these structural families override the
     * method to false for every computation type.
     */
    public static boolean isWaterPathfindable(Block block) {
        if (!hasWaterFluid(block)) return false;
        if (isFence(block) || isFenceGate(block) || contains(BARS, block)) {
            return false;
        }
        // Glass panes need no case of their own: every baseline pane is in
        // LAND_NEVER_IDS, which is consulted below and reaches the same
        // answer. A name-suffix test here would additionally have caught
        // unrelated modded blocks ending in "_glass_pane".
        // SlabBlock permits WATER traversal when waterlogged, whereas
        // StairBlock overrides pathfindability to false for every computation
        // type despite carrying water fluid.
        if (contains(STAIRS, block)) return false;
        if (contains(SLABS, block)) return true;
        if (contains(LAND_NEVER_BLOCKS, block)) return false;
        for (BitSet tag : LAND_NEVER_MEMBERS) {
            if (contains(tag, block)) return false;
        }
        return true;
    }

    private static BitSet tag(String name) {
        return BlockTagIndex.membership(name);
    }

    private static BitSet ids(Set<String> names) {
        BitSet bits = new BitSet();
        for (String name : names) {
            Block block = Block.fromKey(name);
            // Navigation data may lead the linked Minestom registry. An
            // absent block cannot appear in this runtime and needs no bit.
            if (block != null) bits.set(block.id());
        }
        return bits;
    }

    private static boolean contains(BitSet tag, Block block) {
        return tag.get(block.id());
    }

    private static String property(
            Block block, String name, String defaultValue) {
        String value = block.getProperty(name);
        return value == null ? defaultValue : value;
    }
}
