package ca.atlasengine.pathfinding.terrain;

import net.minestom.server.instance.block.Block;

import java.util.Map;

/**
 * An integration's own block-to-{@link TerrainType} mapping.
 *
 * <p>The baseline types are the vocabulary every malus table, corner-cut rule,
 * diagonal rule, and platform-jump landing rule already speaks. A modded or
 * custom block names itself in that vocabulary here rather than by adding a
 * type, so a profile that already prices {@link TerrainType#DAMAGING} prices
 * the new block the moment it is classified as one. Returning {@code null}
 * declines and falls through to the baseline classifier, so an implementation
 * only describes the blocks it owns.</p>
 *
 * <p>Adaptive regions are keyed on the profile carrying this hook, so two
 * entities only reuse one computed route while their classifications compare
 * equal. An implementation that keeps the inherited identity equality, as a
 * lambda does, is never merged with a second instance: that costs route
 * sharing but can never hand an entity a plan computed under different terrain
 * semantics. {@link #keyed} and {@link #equalityKey()} let such an
 * implementation opt back into merging by naming the state it captured, on the
 * same contract as {@code NavigationInfluence}.</p>
 *
 * <p>An answer has to depend on nothing but the block. One search asks a hook
 * at most once per distinct block state and reuses that answer for every cell
 * it classifies, so even an implementation that answers inconsistently decides
 * a whole search under one set of answers.</p>
 */
@FunctionalInterface
public interface TerrainClassification {
    /** Declines every block, leaving the baseline classifier untouched. */
    TerrainClassification NONE = block -> null;

    /** The type of {@code block}, or {@code null} to decline it. */
    TerrainType classify(Block block);

    /**
     * The immutable value deciding which other instances of this exact
     * implementation class may share one computed route. The default is the
     * classification itself, so an implementation that says nothing merges by
     * its own {@code equals} and nothing changes for it.
     *
     * <p>A key is only ever compared against a key declared by the same
     * runtime class, so an unrelated implementation that happens to name the
     * same value can never be merged with this one.</p>
     */
    default Object equalityKey() {
        return this;
    }

    /**
     * Attaches an equality key to a classification that cannot carry one
     * itself. A lambda cannot override {@link #equalityKey()}, so a factory
     * building one wraps it here with a record of everything it captured.
     */
    static TerrainClassification keyed(
            TerrainClassification classification, Object key) {
        return new KeyedTerrainClassification(classification, key);
    }

    /**
     * Names blocks with Minestom's own {@link Block} constants.
     *
     * <pre>{@code
     * TerrainClassification.ofBlocks(Map.of(
     *         Block.SLIME_BLOCK, TerrainType.STICKY_HONEY,
     *         Block.SCULK,       TerrainType.DAMAGING));
     * }</pre>
     *
     * <p>Blocks are matched by identity, ignoring block state, so naming
     * {@link Block#WHEAT} covers wheat at every age. A plain
     * {@code Map<Block, TerrainType>} lookup would not: block equality is
     * state-sensitive, so grown wheat is not equal to the constant.</p>
     *
     * <p>The map is the equality key, so two mobs given the same mapping may
     * share a computed route without anyone stamping a version string.</p>
     */
    static TerrainClassification ofBlocks(Map<Block, TerrainType> blocks) {
        if (blocks == null || blocks.containsKey(null)
                || blocks.containsValue(null)) {
            throw new IllegalArgumentException("block classification");
        }
        return new BlockTerrainClassification(blocks);
    }
}
