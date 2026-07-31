package ca.atlasengine.pathfinding.adaptive;

import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.profile.NavigationProfile;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.concurrent.ConcurrentHashMap;
import net.minestom.server.collision.BoundingBox;

/**
 * Immutable directed mesh of collision-validated route segments.
 *
 * <p>A target field performs reverse Dijkstra once. Afterwards every source
 * query follows stored next edges without searching the world again.</p>
 */
public final class SharedRouteMesh<K> {
    private final long worldRevision;
    private final Map<K, List<Edge<K>>> incoming;
    private final Map<K, List<Edge<K>>> outgoing;

    private SharedRouteMesh(Builder<K> builder) {
        worldRevision = builder.worldRevision;
        incoming = copy(builder.incoming);
        outgoing = copy(builder.outgoing);
    }

    public static <K> Builder<K> builder(long worldRevision) {
        return new Builder<>(worldRevision);
    }

    public long worldRevision() {
        return worldRevision;
    }

    public int nodeCount() {
        return outgoing.size();
    }

    public TargetField<K> routesTo(K target, long currentWorldRevision) {
        if (target == null || currentWorldRevision != worldRevision) {
            throw new IllegalArgumentException("target or world revision");
        }
        if (!outgoing.containsKey(target)) {
            throw new IllegalArgumentException("target is not in mesh");
        }
        return new TargetField<>(this, target);
    }

    private static <K> Map<K, List<Edge<K>>> copy(
            Map<K, List<Edge<K>>> source) {
        Map<K, List<Edge<K>>> result = new HashMap<>();
        source.forEach((key, value) -> result.put(key, List.copyOf(value)));
        return Map.copyOf(result);
    }

    public static final class Builder<K> {
        private final long worldRevision;
        private final Map<K, List<Edge<K>>> incoming = new HashMap<>();
        private final Map<K, List<Edge<K>>> outgoing = new HashMap<>();
        private BoundingBox boundingBox;
        private NavigationProfile profile;

        private Builder(long worldRevision) {
            this.worldRevision = worldRevision;
        }

        public Builder<K> node(K key) {
            if (key == null) throw new IllegalArgumentException("key");
            incoming.computeIfAbsent(key, ignored -> new ArrayList<>());
            outgoing.computeIfAbsent(key, ignored -> new ArrayList<>());
            return this;
        }

        /** Adds one directed, physically validated segment and its total cost. */
        public Builder<K> route(
                K from, K to, NavigationPlan plan, double cost) {
            if (from == null || to == null || from.equals(to)
                    || plan == null || !plan.usable()
                    || !Double.isFinite(cost) || cost < 0) {
                throw new IllegalArgumentException("invalid directed route");
            }
            if (profile == null) {
                profile = plan.profile();
                boundingBox = plan.boundingBox();
            } else if (!profile.equals(plan.profile())
                    || !boundingBox.equals(plan.boundingBox())) {
                throw new IllegalArgumentException(
                        "mesh routes require one navigation profile and bounding box");
            }
            node(from);
            node(to);
            Edge<K> edge = new Edge<>(from, to, plan, cost);
            outgoing.get(from).add(edge);
            incoming.get(to).add(edge);
            return this;
        }

        public SharedRouteMesh<K> build() {
            if (outgoing.isEmpty()) throw new IllegalStateException("empty mesh");
            return new SharedRouteMesh<>(this);
        }
    }

    public static final class TargetField<K> {
        private final K target;
        private final Map<K, Edge<K>> next;
        private final Map<K, Double> distances;
        private final ConcurrentHashMap<K, Optional<SharedNavigationRoute<K>>> routes =
                new ConcurrentHashMap<>();

        private TargetField(SharedRouteMesh<K> mesh, K target) {
            this.target = target;
            Map<K, Edge<K>> mutableNext = new HashMap<>();
            Map<K, Double> mutableDistances = new HashMap<>();
            PriorityQueue<Distance<K>> open = new PriorityQueue<>();
            mutableDistances.put(target, 0.0);
            open.add(new Distance<>(target, 0));
            while (!open.isEmpty()) {
                Distance<K> current = open.poll();
                if (current.cost > mutableDistances.getOrDefault(
                        current.key, Double.POSITIVE_INFINITY)) continue;
                for (Edge<K> incoming : mesh.incoming.getOrDefault(
                        current.key, List.of())) {
                    double candidate = current.cost + incoming.cost;
                    if (candidate >= mutableDistances.getOrDefault(
                            incoming.from, Double.POSITIVE_INFINITY)) continue;
                    mutableDistances.put(incoming.from, candidate);
                    mutableNext.put(incoming.from, incoming);
                    open.add(new Distance<>(incoming.from, candidate));
                }
            }
            next = Map.copyOf(mutableNext);
            distances = Map.copyOf(mutableDistances);
        }

        public K target() {
            return target;
        }

        public boolean canReach(K source) {
            return source != null && distances.containsKey(source);
        }

        public Optional<SharedNavigationRoute<K>> routeFrom(K source) {
            if (source == null || source.equals(target)
                    || !distances.containsKey(source)) return Optional.empty();
            Optional<SharedNavigationRoute<K>> cached = routes.get(source);
            if (cached != null) return cached;
            // Keep atomic first construction for concurrent callers, but do not
            // create the capturing mapping function on every cache hit.
            return routes.computeIfAbsent(
                    source, key -> Optional.of(buildRoute(key)));
        }

        public double costFrom(K source) {
            return distances.getOrDefault(source, Double.POSITIVE_INFINITY);
        }

        private SharedNavigationRoute<K> buildRoute(K source) {
            List<NavigationPlan> segments = new ArrayList<>();
            K cursor = source;
            int maximumSegments = next.size() + 1;
            while (!cursor.equals(target) && segments.size() < maximumSegments) {
                Edge<K> edge = next.get(cursor);
                if (edge == null) throw new IllegalStateException("broken target field");
                segments.add(edge.plan);
                cursor = edge.to;
            }
            if (!cursor.equals(target)) throw new IllegalStateException("route cycle");
            return new SharedNavigationRoute<>(
                    source, target, segments, distances.get(source));
        }
    }

    private record Edge<K>(K from, K to, NavigationPlan plan, double cost) {
    }

    private record Distance<K>(K key, double cost)
            implements Comparable<Distance<K>> {
        @Override
        public int compareTo(Distance<K> other) {
            return Double.compare(cost, other.cost);
        }
    }
}
