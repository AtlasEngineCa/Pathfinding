package ca.atlasengine.pathfinding.terrain;

import net.kyori.adventure.key.Key;
import net.minestom.server.instance.block.Block;
import net.minestom.server.registry.RegistryKey;
import net.minestom.server.registry.RegistryTag;

import java.util.BitSet;

/**
 * Block-tag membership read from Minestom's block registry.
 *
 * <p>Tags come from {@link Block#staticRegistry()}, so membership is whatever
 * the linked Minestom version says it is. Nothing here parses a bundled copy
 * of the tag data or falls back to name-pattern heuristics.</p>
 *
 * <p>A tag is named by its {@link Key}. The short form is accepted for
 * convenience and resolves in the {@code minecraft} namespace, which is the
 * namespace used by baseline block tags.</p>
 */
public final class BlockTagIndex {
    private BlockTagIndex() {
    }

    /** Whether {@code block} carries {@code tag}, compared by identity. */
    public static boolean contains(Key tag, Block block) {
        RegistryTag<Block> members = Block.staticRegistry().getTag(tag);
        if (members == null) return false;
        RegistryKey<Block> key = Block.staticRegistry().getKey(block);
        return key != null && members.contains(key);
    }

    /** Short-name overload resolving in the {@code minecraft} namespace. */
    public static boolean contains(String tag, Block block) {
        return contains(key(tag), block);
    }

    /**
     * Hot-path representation of one tag, built once by the caller.
     *
     * <p>Membership is tested once per expanded node, so the registry lookup
     * is collapsed into a bitset indexed by block id. Those ids are an
     * internal detail of this cache and never leave the package.</p>
     */
    static BitSet membership(String tag) {
        BitSet bits = new BitSet();
        RegistryTag<Block> members = Block.staticRegistry().getTag(key(tag));
        if (members == null) return bits;
        for (RegistryKey<Block> member : members) {
            Block block = Block.staticRegistry().get(member);
            // Navigation data may lead the linked Minestom registry.
            if (block != null) bits.set(block.id());
        }
        return bits;
    }

    private static Key key(String tag) {
        return tag.indexOf(':') < 0 ? Key.key("minecraft", tag) : Key.key(tag);
    }
}
