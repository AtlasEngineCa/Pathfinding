package ca.atlasengine.pathfinding;

import net.minestom.server.coordinate.Point;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;

/**
 * Bounded least-recently-used storage for repeatedly traversed routes.
 *
 * <p>The integrating world owns the monotonically increasing revision. A
 * cached plan is returned only for the exact revision under which it was
 * computed, preventing replay after relevant world changes.</p>
 */
public final class NavigationPlanCache<K> {
    private final int capacity;
    private final Map<K, Entry> plans;

    public NavigationPlanCache(int capacity) {
        if (capacity <= 0) throw new IllegalArgumentException("capacity");
        this.capacity = capacity;
        plans = new LinkedHashMap<>(16, 0.75f, true);
    }

    public synchronized void put(K key, long worldRevision,
                                 NavigationPlan plan) {
        if (key == null || plan == null || !plan.usable()) {
            throw new IllegalArgumentException("key and usable plan required");
        }
        plans.put(key, new Entry(worldRevision, plan));
        while (plans.size() > capacity) {
            plans.remove(plans.keySet().iterator().next());
        }
    }

    public synchronized Optional<NavigationPlan> get(
            K key, long worldRevision) {
        if (key == null) return Optional.empty();
        Entry entry = plans.get(key);
        if (entry == null) return Optional.empty();
        if (entry.worldRevision != worldRevision) {
            plans.remove(key);
            return Optional.empty();
        }
        return Optional.of(entry.plan);
    }

    /** Also checks that the replaying entity begins near the stored start. */
    public synchronized Optional<NavigationPlan> get(
            K key, long worldRevision, Point currentStart,
            double maximumStartDistance) {
        if (currentStart == null || !Double.isFinite(maximumStartDistance)
                || maximumStartDistance < 0) return Optional.empty();
        return get(key, worldRevision).filter(plan ->
                plan.start().distance(currentStart) <= maximumStartDistance);
    }

    public synchronized void invalidate(K key) {
        plans.remove(key);
    }

    public synchronized void clear() {
        plans.clear();
    }

    public synchronized int size() {
        return plans.size();
    }

    public int capacity() {
        return capacity;
    }

    private record Entry(long worldRevision, NavigationPlan plan) {
    }
}
