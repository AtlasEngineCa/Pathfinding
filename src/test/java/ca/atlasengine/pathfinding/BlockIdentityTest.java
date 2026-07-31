package ca.atlasengine.pathfinding;

import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Block identity is compared with {@link Block#compare(Block)} rather than by
 * name, so this pins the two properties the terrain data relies on: state is
 * ignored, and a different block never matches.
 */
class BlockIdentityTest {
    @Test
    void comparisonIgnoresBlockState() {
        assertTrue(Block.LADDER.withProperty("facing", "west")
                .compare(Block.LADDER));
        assertTrue(Block.SNOW.withProperty("layers", "4").compare(Block.SNOW));
        assertTrue(Block.WATER.withProperty("level", "3").compare(Block.WATER));
    }

    @Test
    void comparisonDistinguishesDifferentBlocks() {
        assertFalse(Block.IRON_DOOR.compare(Block.OAK_DOOR));
        assertFalse(Block.LAVA.compare(Block.WATER));
    }
}
