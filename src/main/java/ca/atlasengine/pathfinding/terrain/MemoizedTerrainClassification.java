package ca.atlasengine.pathfinding.terrain;

import net.minestom.server.instance.block.Block;

import java.util.IdentityHashMap;
import java.util.Map;

/**
 * Asks its delegate at most once per distinct block state and answers every
 * later query for that state from the memo. One search holds one of these, so
 * the classification a search starts with is the one it finishes with.
 *
 * <p>{@code Block.Getter.Condition.TYPE} hands back the interned state
 * instance, so identity both hits and stays cheap. A miss only costs one more
 * delegate call.</p>
 */
final class MemoizedTerrainClassification implements TerrainClassification {
    private static final Object DECLINED = new Object();

    private final TerrainClassification delegate;
    private final Map<Block, Object> memo = new IdentityHashMap<>();

    MemoizedTerrainClassification(TerrainClassification delegate) {
        this.delegate = delegate;
    }

    @Override
    public TerrainType classify(Block block) {
        Object memoized = memo.get(block);
        if (memoized == null) {
            TerrainType resolved = delegate.classify(block);
            memo.put(block, resolved == null ? DECLINED : resolved);
            return resolved;
        }
        return memoized == DECLINED ? null : (TerrainType) memoized;
    }

    @Override
    public Object equalityKey() {
        return delegate.equalityKey();
    }

    @Override
    public boolean equals(Object object) {
        return object instanceof MemoizedTerrainClassification other
                && delegate.equals(other.delegate);
    }

    @Override
    public int hashCode() {
        return delegate.hashCode();
    }
}
