package ca.atlasengine.pathfinding.terrain;

import net.minestom.server.instance.block.Block;

/**
 * Classifies exactly like its delegate and compares on the delegate's declared
 * equality key.
 *
 * <p>The delegate's runtime class is half of the comparison. Two unrelated
 * implementations that name the same key therefore stay apart, and so do two
 * lambdas written at two source sites, because the JVM spins one class per
 * lambda expression. Only repeated instances of one implementation, which is
 * what a factory produces, can merge.</p>
 */
record KeyedTerrainClassification(TerrainClassification delegate, Object key)
        implements TerrainClassification {
    KeyedTerrainClassification {
        if (delegate == null || key == null) {
            throw new IllegalArgumentException("keyed classification");
        }
    }

    @Override
    public TerrainType classify(Block block) {
        return delegate.classify(block);
    }

    @Override
    public boolean equals(Object object) {
        return object instanceof KeyedTerrainClassification other
                && delegate.getClass() == other.delegate.getClass()
                && key.equals(other.key);
    }

    @Override
    public int hashCode() {
        return 31 * delegate.getClass().hashCode() + key.hashCode();
    }
}
