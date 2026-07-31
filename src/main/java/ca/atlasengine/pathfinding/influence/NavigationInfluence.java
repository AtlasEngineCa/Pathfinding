package ca.atlasengine.pathfinding.influence;

import ca.atlasengine.pathfinding.search.SearchControl;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block;

import java.util.ArrayList;
import java.util.List;

/**
 * A snapshot-safe dynamic navigation rule.
 *
 * <p>Adaptive regions are keyed on the influences of a request, so two
 * entities only reuse one computed route while their influences compare
 * equal. Every baseline influence implements value equality over exactly the
 * fields {@link #evaluate} reads. An implementation that keeps the inherited
 * identity equality, as a lambda does, is never merged with a second
 * instance: that costs route sharing but can never hand an entity a plan
 * computed under different rules. {@link #keyed} and {@link #equalityKey()}
 * let such an implementation opt back into merging by naming the state it
 * captured.</p>
 */
@FunctionalInterface
public interface NavigationInfluence {
    InfluenceResult evaluate(Block.Getter blocks, Point point, BoundingBox boundingBox);

    /** Cooperative variant for influences whose evaluation may scan blocks. */
    default InfluenceResult evaluate(
            Block.Getter blocks, Point point, BoundingBox boundingBox,
            SearchControl control) {
        return evaluate(blocks, point, boundingBox);
    }

    /**
     * The immutable value deciding which other instances of this exact
     * implementation class may share one computed route. The default is the
     * influence itself, so an implementation that says nothing merges by its
     * own {@code equals} and nothing changes for it.
     *
     * <p>A key is only ever compared against a key declared by the same
     * runtime class, so an unrelated implementation that happens to name the
     * same value can never be merged with this one. Naming a key that leaves
     * out state {@link #evaluate} reads stays as wrong, and as impossible to
     * enforce, as writing an {@code equals} that leaves out a field.</p>
     */
    default Object equalityKey() {
        return this;
    }

    /**
     * Attaches an equality key to an influence that cannot carry one itself.
     * A lambda cannot override {@link #equalityKey()}, so a factory building
     * one wraps it here with a record of everything it captured.
     */
    static NavigationInfluence keyed(
            NavigationInfluence influence, Object key) {
        return new KeyedInfluence(influence, key);
    }

    /**
     * Returns {@code influences} with every element that declares a key other
     * than itself replaced by a delegate comparing on that key. The argument
     * is returned untouched when no element declares one, so a list of
     * baselines or of bare lambdas compares exactly as it always has.
     */
    static List<NavigationInfluence> byDeclaredEquality(
            List<NavigationInfluence> influences) {
        if (influences == null) throw new IllegalArgumentException("influences");
        List<NavigationInfluence> keyed = null;
        for (int index = 0; index < influences.size(); index++) {
            NavigationInfluence influence = influences.get(index);
            Object key = influence.equalityKey();
            boolean declared = key != null && key != influence;
            if (declared && keyed == null) {
                keyed = new ArrayList<>(influences.subList(0, index));
            }
            if (keyed != null) {
                keyed.add(declared
                        ? new KeyedInfluence(influence, key) : influence);
            }
        }
        return keyed == null ? influences : List.copyOf(keyed);
    }
}
