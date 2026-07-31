package ca.atlasengine.pathfinding.terrain;

import net.minestom.server.instance.block.Block;

import java.util.HashMap;
import java.util.Map;

/**
 * A {@link TerrainClassification} over Minestom block constants.
 *
 * <p>Lookup is by block id rather than by the block itself, because block
 * equality includes state: {@code Block.WHEAT} is not equal to wheat at age
 * three, while both share an id.</p>
 */
final class BlockTerrainClassification implements TerrainClassification {
    private final Map<Block, TerrainType> declared;
    private final Map<Integer, TerrainType> byId;

    BlockTerrainClassification(Map<Block, TerrainType> blocks) {
        declared = Map.copyOf(blocks);
        Map<Integer, TerrainType> ids = new HashMap<>(declared.size());
        for (Map.Entry<Block, TerrainType> entry : declared.entrySet()) {
            ids.put(entry.getKey().id(), entry.getValue());
        }
        byId = Map.copyOf(ids);
    }

    @Override
    public TerrainType classify(Block block) {
        return byId.get(block.id());
    }

    @Override
    public Object equalityKey() {
        return declared;
    }

    @Override
    public boolean equals(Object object) {
        return object instanceof BlockTerrainClassification other
                && declared.equals(other.declared);
    }

    @Override
    public int hashCode() {
        return declared.hashCode();
    }

    @Override
    public String toString() {
        return "BlockTerrainClassification" + declared;
    }
}
