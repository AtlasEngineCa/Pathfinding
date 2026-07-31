package ca.atlasengine.pathfinding.influence;

import ca.atlasengine.pathfinding.search.SearchControl;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block;

/**
 * Evaluates exactly like its delegate and compares on the delegate's declared
 * equality key.
 *
 * <p>The delegate's runtime class is half of the comparison. Two unrelated
 * implementations that name the same key therefore stay apart, and so do two
 * lambdas written at two source sites, because the JVM spins one class per
 * lambda expression. Only repeated instances of one implementation, which is
 * what a factory produces, can merge.</p>
 */
record KeyedInfluence(NavigationInfluence delegate, Object key)
        implements NavigationInfluence {
    KeyedInfluence {
        if (delegate == null || key == null) {
            throw new IllegalArgumentException("keyed influence");
        }
    }

    @Override
    public InfluenceResult evaluate(
            Block.Getter blocks, Point point, BoundingBox boundingBox) {
        return delegate.evaluate(blocks, point, boundingBox);
    }

    @Override
    public InfluenceResult evaluate(
            Block.Getter blocks, Point point, BoundingBox boundingBox,
            SearchControl control) {
        return delegate.evaluate(blocks, point, boundingBox, control);
    }

    @Override
    public boolean equals(Object object) {
        return object instanceof KeyedInfluence other
                && delegate.getClass() == other.delegate.getClass()
                && key.equals(other.key);
    }

    @Override
    public int hashCode() {
        return 31 * delegate.getClass().hashCode() + key.hashCode();
    }
}
