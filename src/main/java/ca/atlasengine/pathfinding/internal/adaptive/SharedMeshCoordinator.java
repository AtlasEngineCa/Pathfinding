package ca.atlasengine.pathfinding.internal.adaptive;

import ca.atlasengine.pathfinding.adaptive.SharedMeshPolicy;
import ca.atlasengine.pathfinding.adaptive.SharedMeshHandle;
import ca.atlasengine.pathfinding.adaptive.SharedMeshSource;
import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.adaptive.SharedMeshRetention;
import ca.atlasengine.pathfinding.adaptive.SharedNavigationRoute;
import ca.atlasengine.pathfinding.adaptive.SharedRouteMesh;
import ca.atlasengine.pathfinding.search.EntityTraversalState;
import ca.atlasengine.pathfinding.metrics.NavigationMetrics;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.result.PathNodeCost;
import ca.atlasengine.pathfinding.result.PathStatus;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.CompletableFuture;

/**
 * Observes compatible requests and promotes repeated traffic to regional,
 * revisioned target fields while retaining ordinary A* as the fallback.
 *
 * <p>The coordinator owns only routing metadata. Callers own returned handles
 * and must close them when an entity dies, despawns, changes instance, or no
 * longer follows that target. Call {@link #tick(long)} from the owning tick
 * thread to apply idle expiry and promotion hysteresis.</p>
 */
public final class SharedMeshCoordinator implements AutoCloseable {
    @FunctionalInterface
    public interface PlanSubmitter {
        CompletableFuture<NavigationPlan> submit(
                Object actorKey, NavigationRequest request);
    }

    private final PlanSubmitter submitter;
    private final SharedMeshPolicy config;
    private final NavigationMetrics metrics;
    private final LinkedHashMap<RegionKey, Region> regions =
            new LinkedHashMap<>(16, 0.75f, true);
    private final Map<Object, Membership> actors = new HashMap<>();
    private final Map<Object, CompletableFuture<NavigationPlan>> pendingSearches =
            new HashMap<>();
    private long sequence;
    private boolean closed;

    public SharedMeshCoordinator(
            PlanSubmitter submitter, SharedMeshPolicy config) {
        this(submitter, config, new NavigationMetrics());
    }

    /** Records dispatch decisions into a recorder shared with a service. */
    public SharedMeshCoordinator(
            PlanSubmitter submitter, SharedMeshPolicy config,
            NavigationMetrics metrics) {
        if (submitter == null || config == null || metrics == null) {
            throw new IllegalArgumentException("submitter, config and metrics");
        }
        this.submitter = submitter;
        this.config = config;
        this.metrics = metrics;
    }

    /** Cumulative telemetry, including the mesh reuse breakdown. */
    public NavigationMetrics metrics() {
        return metrics;
    }

    /**
     * Requests a plan and automatically replaces any previous membership for
     * the same actor. The target key should be the stable identity of a moving
     * entity; its current block position is tracked separately by the request.
     */
    public SharedMeshHandle request(
            Object actorKey, Object targetKey, long worldRevision,
            long currentTick, NavigationRequest request) {
        if (actorKey == null || targetKey == null || request == null
                || currentTick < 0) {
            throw new IllegalArgumentException("invalid adaptive request");
        }

        final long membershipId;
        final TargetNode targetNode = new TargetNode(
                targetKey, TargetCell.of(request.target()));
        final SourceNode sourceNode = new SourceNode(
                ExactPosition.of(request.start()),
                StartCompatibility.of(request.entityState()));
        final Dispatch dispatch;

        synchronized (this) {
            ensureOpen();
            removeActorInternal(actorKey);
            Region selected = regionFor(
                    RegionKey.of(request, config), worldRevision, currentTick);
            if (selected == null || !admitsTarget(selected, targetNode)) {
                return untrackedFallback(actorKey, targetKey, request);
            }
            membershipId = join(actorKey, selected, targetNode, currentTick);
            dispatch = dispatch(actorKey, selected, worldRevision,
                    sourceNode, targetNode, request, currentTick);
        }

        return new SharedMeshHandle(
                actorKey, targetKey, dispatch.source(), dispatch.future(),
                () -> closeMembership(actorKey, membershipId));
    }

    private Region regionFor(
            RegionKey regionKey, long worldRevision, long currentTick) {
        Region existing = regions.get(regionKey);
        if (existing == null) {
            if (regions.size() >= config.maximumRegions()) return null;
            Region created = new Region(regionKey, worldRevision, currentTick);
            regions.put(regionKey, created);
            return created;
        }
        if (existing.worldRevision != worldRevision) {
            existing.reset(worldRevision, currentTick);
        }
        return existing;
    }

    /**
     * A pursued entity re-enters cells constantly, so a full region evicts its
     * least recently used unoccupied cell rather than dropping the request to
     * untracked A*. Occupied cells are never evicted, so the hard bound still
     * refuses a genuinely oversubscribed region.
     */
    private boolean admitsTarget(Region region, TargetNode targetNode) {
        if (region.targets.containsKey(targetNode)
                || region.targets.size() < config.maximumTargetsPerRegion()) {
            return true;
        }
        TargetNode evicted = null;
        for (Map.Entry<TargetNode, TargetUsage> entry
                : region.targets.entrySet()) {
            if (entry.getValue().members == 0) {
                evicted = entry.getKey();
                break;
            }
        }
        if (evicted == null) return false;
        region.forget(evicted);
        return true;
    }

    private long join(
            Object actorKey, Region region, TargetNode targetNode,
            long currentTick) {
        region.lastRequestTick = currentTick;
        region.requestCount++;
        TargetUsage usage = region.targets.computeIfAbsent(
                targetNode, ignored -> new TargetUsage(currentTick));
        usage.members++;
        usage.lastUsedTick = currentTick;
        region.activeMembers++;
        long membershipId = ++sequence;
        actors.put(actorKey, new Membership(
                membershipId, region, targetNode));
        return membershipId;
    }

    private Dispatch dispatch(
            Object actorKey, Region region, long worldRevision,
            SourceNode sourceNode, TargetNode targetNode,
            NavigationRequest request, long currentTick) {
        // A region stores one plan per destination class, so re-anchor it on
        // this request's endpoints. Last line of defence: a published plan
        // only reaches a follower when it still matches the request that is
        // being answered, including the exact, never-merged source.
        Optional<NavigationPlan> shared = region.sharedPlan(
                        sourceNode, targetNode)
                .map(plan -> plan.withTarget(request.target()))
                .filter(plan -> NavigationPlanParity.requestCompatible(
                        request, plan));
        if (shared.isPresent()) {
            metrics.adaptiveDispatch(
                    SharedMeshSource.SHARED_TARGET_FIELD);
            return new Dispatch(
                    SharedMeshSource.SHARED_TARGET_FIELD,
                    CompletableFuture.completedFuture(shared.orElseThrow()));
        }
        // Read before submitting: an already-completed future runs acceptPlan
        // inline on this thread and may promote the region.
        SharedMeshSource source = region.certifying()
                ? SharedMeshSource.PARITY_CERTIFICATION_SEARCH
                : SharedMeshSource.INDIVIDUAL_SEARCH;
        metrics.adaptiveDispatch(source);
        CompletableFuture<NavigationPlan> search =
                submitter.submit(actorKey, request);
        pendingSearches.put(actorKey, search);
        CompletableFuture<NavigationPlan> future = search.thenApply(plan -> {
            acceptPlan(region, worldRevision, sourceNode,
                    targetNode, request, plan, currentTick);
            return plan;
        });
        search.whenComplete((ignored, failure) ->
                clearPending(actorKey, search));
        return new Dispatch(source, future);
    }

    private record Dispatch(
            SharedMeshSource source,
            CompletableFuture<NavigationPlan> future) {
    }

    /** Applies idle target expiry, demotion hysteresis, and empty-region cleanup. */
    public synchronized void tick(long currentTick) {
        if (currentTick < 0) throw new IllegalArgumentException("currentTick");
        if (closed) return;
        Iterator<Map.Entry<RegionKey, Region>> iterator =
                regions.entrySet().iterator();
        while (iterator.hasNext()) {
            Region region = iterator.next().getValue();
            boolean graphChanged = expireIdleTargets(region, currentTick);
            applyDemotionHysteresis(region, currentTick, graphChanged);
            if (regionExpired(region, currentTick)) iterator.remove();
        }
    }

    private boolean expireIdleTargets(Region region, long currentTick) {
        List<TargetNode> expired = new ArrayList<>();
        region.targets.forEach((target, usage) -> {
            if (usage.members == 0
                    && currentTick - usage.lastUsedTick
                    >= config.targetIdleTicks()) expired.add(target);
        });
        boolean graphChanged = false;
        for (TargetNode target : expired) graphChanged |= region.forget(target);
        return graphChanged;
    }

    private void applyDemotionHysteresis(
            Region region, long currentTick, boolean graphChanged) {
        if (region.certifying()
                && region.activeMembers < config.retentionRequests()
                && currentTick - region.lastRequestTick
                >= config.targetIdleTicks()) {
            region.promoted = false;
            region.promoting = false;
            region.fields.clear();
            region.fieldEdges = Set.of();
            region.publishedPlans.clear();
            region.requestCount = region.activeMembers;
            region.invalidate();
        } else if (graphChanged && region.certifying()) {
            region.rebuild(config);
        }
    }

    /**
     * Builds a region's mesh and target fields from an immutable snapshot.
     * Nothing here reads coordinator state, so a caller may run it without
     * holding the monitor and hand the result to {@link Region#publish}.
     *
     * <p>A target field is a reverse search over the edges that reach one
     * destination, so it can only change when an added or removed edge ends on
     * a node that already reaches it. A field whose reachable set contains no
     * such endpoint still describes the new graph exactly and is carried over
     * instead of being searched again.</p>
     */
    private static RebuildResult computeRebuild(
            RebuildSnapshot snapshot, SharedMeshPolicy config) {
        Set<EdgeKey> edges = snapshot.edges().keySet();
        if (!snapshot.promoted() || edges.isEmpty()) {
            return new RebuildResult(Map.of(), Set.copyOf(edges), Map.of());
        }
        Set<MeshNode> touched = new HashSet<>();
        for (EdgeKey edge : edges) {
            if (!snapshot.fieldEdges().contains(edge)) touched.add(edge.to);
        }
        for (EdgeKey edge : snapshot.fieldEdges()) {
            if (!edges.contains(edge)) touched.add(edge.to);
        }
        Map<TargetNode, SharedRouteMesh.TargetField<MeshNode>> fields =
                new HashMap<>();
        List<TargetNode> stale = new ArrayList<>();
        for (TargetNode target : snapshot.targets()) {
            SharedRouteMesh.TargetField<MeshNode> previous =
                    snapshot.fields().get(target);
            if (previous != null && unaffected(previous, touched)) {
                fields.put(target, previous);
            } else {
                stale.add(target);
            }
        }
        if (!stale.isEmpty()) {
            SharedRouteMesh<MeshNode> mesh;
            try {
                SharedRouteMesh.Builder<MeshNode> builder =
                        SharedRouteMesh.builder(snapshot.worldRevision());
                snapshot.edges().forEach((edge, plan) -> builder.route(
                        edge.from, edge.to, plan, graphLength(plan.nodes())));
                mesh = builder.build();
            } catch (IllegalArgumentException | IllegalStateException ignored) {
                // The retained edges do not form a buildable mesh. Every
                // request in this region keeps performing ordinary A*.
                return new RebuildResult(Map.of(), Set.copyOf(edges), Map.of());
            }
            int retainedTargets = fields.size();
            for (TargetNode target : stale) {
                if (retainedTargets++ >= config.maximumTargetsPerRegion()) break;
                try {
                    fields.put(target,
                            mesh.routesTo(target, snapshot.worldRevision()));
                } catch (IllegalArgumentException ignored) {
                    // No validated edge reaches this target position yet.
                }
            }
        }
        Map<ObservationKey, Published> published = new HashMap<>();
        for (Map.Entry<ObservationKey, NavigationPlan> entry
                : snapshot.certified().entrySet()) {
            ObservationKey key = entry.getKey();
            SharedRouteMesh.TargetField<MeshNode> field =
                    fields.get(key.target);
            if (field == null) continue;
            Published previous = snapshot.published().get(key);
            if (previous != null && previous.certificate() == entry.getValue()
                    && field == snapshot.fields().get(key.target)) {
                published.put(key, previous);
                continue;
            }
            try {
                // A charged cost belongs to one whole search: reaching a
                // cell by another route can classify or price it
                // differently. The composition proves the mesh reproduces
                // the certified route; the certificate prices it.
                field.routeFrom(key.source)
                        .map(SharedNavigationRoute::plan)
                        .filter(plan -> NavigationPlanParity
                                .semanticallyEquivalent(
                                        entry.getValue(), plan))
                        .map(plan -> plan.withNodeCosts(
                                entry.getValue().nodeCosts()))
                        .ifPresent(plan -> published.put(
                                key, new Published(entry.getValue(), plan)));
            } catch (IllegalStateException | IllegalArgumentException ignored) {
                // A malformed or lossy composition is not published. Its
                // next request automatically performs ordinary A*.
            }
        }
        return new RebuildResult(
                Map.copyOf(fields), Set.copyOf(edges), Map.copyOf(published));
    }

    private static boolean unaffected(
            SharedRouteMesh.TargetField<MeshNode> field,
            Set<MeshNode> touched) {
        for (MeshNode node : touched) {
            if (field.canReach(node)) return false;
        }
        return true;
    }

    private boolean regionExpired(Region region, long currentTick) {
        return region.activeMembers == 0
                && currentTick - region.lastRequestTick
                >= config.regionIdleTicks();
    }

    /** Removes one actor immediately; safe to call in addition to handle close. */
    public synchronized void removeActor(Object actorKey) {
        if (actorKey != null) removeActorInternal(actorKey);
    }

    /** Removes every membership and target field for a despawned target. */
    public synchronized void removeTarget(Object targetKey) {
        if (targetKey == null) return;
        List<Object> removedActors = new ArrayList<>();
        actors.forEach((actor, membership) -> {
            if (membership.target.targetKey.equals(targetKey)) {
                removedActors.add(actor);
            }
        });
        removedActors.forEach(this::removeActorInternal);
        for (Region region : regions.values()) {
            List<TargetNode> dropped = region.targets.keySet().stream()
                    .filter(target -> target.targetKey.equals(targetKey))
                    .toList();
            boolean changed = false;
            for (TargetNode target : dropped) changed |= region.forget(target);
            if (changed) region.rebuild(config);
        }
    }

    /** Invalidates retained topology for one world after relevant block changes. */
    public synchronized void invalidateWorld(
            Block.Getter blocks, long newWorldRevision, long currentTick) {
        if (blocks == null || currentTick < 0) {
            throw new IllegalArgumentException("blocks and tick");
        }
        for (Region region : regions.values()) {
            if (region.key.world.value == blocks
                    && region.worldRevision != newWorldRevision) {
                region.reset(newWorldRevision, currentTick);
            }
        }
    }

    /** Drops one unloaded regional cell while leaving other cells untouched. */
    public synchronized void unloadRegion(
            Block.Getter blocks, int regionX, int regionY, int regionZ) {
        if (blocks == null) throw new IllegalArgumentException("blocks");
        List<Object> removedActors = new ArrayList<>();
        actors.forEach((actor, membership) -> {
            RegionKey key = membership.region.key;
            if (key.world.value == blocks && key.regionX == regionX
                    && key.regionY == regionY && key.regionZ == regionZ) {
                removedActors.add(actor);
            }
        });
        removedActors.forEach(this::removeActorInternal);
        regions.keySet().removeIf(key -> key.world.value == blocks
                && key.regionX == regionX && key.regionY == regionY
                && key.regionZ == regionZ);
    }

    public synchronized SharedMeshRetention stats() {
        int promoted = 0;
        int nodes = 0;
        int targets = 0;
        int observed = 0;
        int certified = 0;
        int published = 0;
        for (Region region : regions.values()) {
            if (region.promoted) promoted++;
            nodes += region.nodeCount();
            targets += region.targets.size();
            observed += region.observedPlans.size();
            certified += region.certifiedPlans.size();
            published += region.publishedPlans.size();
        }
        return new SharedMeshRetention(
                regions.size(), promoted, actors.size(), targets, nodes,
                observed, certified, published);
    }

    public SharedMeshPolicy config() {
        return config;
    }

    @Override
    public synchronized void close() {
        if (closed) return;
        closed = true;
        // Cancelling runs the search's own completion callback inline, which
        // is reentrant on this monitor and removes its own pending entry.
        // Drain first so that removal cannot mutate a live iteration.
        List<CompletableFuture<NavigationPlan>> pending =
                new ArrayList<>(pendingSearches.values());
        pendingSearches.clear();
        pending.forEach(future -> future.cancel(true));
        actors.clear();
        regions.clear();
    }

    private void acceptPlan(
            Region region, long worldRevision,
            SourceNode source, TargetNode target,
            NavigationRequest request, NavigationPlan plan, long requestTick) {
        RebuildSnapshot snapshot;
        synchronized (this) {
            if (closed || region.worldRevision != worldRevision
                    || plan == null || !plan.usable()
                    || !regions.containsValue(region)
                    || !region.targets.containsKey(target)
                    || !NavigationPlanParity.requestCompatible(request, plan)) return;
            // Retain one canonical plan per destination class. Every request that
            // is answered from it is re-anchored on its own exact endpoints.
            NavigationPlan retained = plan.withTarget(
                    target.cell.representative());
            region.lastRequestTick = Math.max(region.lastRequestTick, requestTick);
            if (!region.certifying()) {
                region.observe(source, target, retained, config);
            }
            if (!region.certifying()
                    && region.requestCount >= config.promotionRequests()) {
                region.promoting = true;
                region.promoteObserved(config);
            } else if (region.certifying()
                    && region.canRetain(source,
                    retained, target, config.maximumNodesPerRegion())) {
                if (!region.certifyAndRetain(source, retained, target)) return;
            } else {
                return;
            }
            // One rebuild per region at a time. A change made while this one
            // runs is folded into its next pass rather than racing a second
            // copy of the same reverse Dijkstra.
            if (region.rebuilding) return;
            region.rebuilding = true;
            snapshot = region.snapshot();
        }
        // Reverse Dijkstra runs outside the monitor so a promotion cannot stall
        // the tick thread. The publish discards the result unless the region is
        // still the one it was computed for, at the same world revision and
        // promotion state, with no intervening topology change.
        try {
            while (snapshot != null) {
                snapshot = publishRebuild(
                        region, snapshot, computeRebuild(snapshot, config));
            }
        } catch (RuntimeException | Error failure) {
            releaseRebuild(region);
            throw failure;
        }
    }

    /** Installs one result and returns the next snapshot if the region moved. */
    private synchronized RebuildSnapshot publishRebuild(
            Region region, RebuildSnapshot snapshot, RebuildResult result) {
        if (!closed && regions.containsValue(region)) {
            region.publish(snapshot, result);
            if (region.topologyVersion != snapshot.version()) {
                return region.snapshot();
            }
        }
        region.rebuilding = false;
        return null;
    }

    private synchronized void releaseRebuild(Region region) {
        region.rebuilding = false;
    }

    private SharedMeshHandle untrackedFallback(
            Object actorKey, Object targetKey, NavigationRequest request) {
        metrics.adaptiveDispatch(
                SharedMeshSource.UNTRACKED_FALLBACK);
        CompletableFuture<NavigationPlan> future =
                submitter.submit(actorKey, request);
        return new SharedMeshHandle(
                actorKey, targetKey,
                SharedMeshSource.UNTRACKED_FALLBACK,
                future, () -> future.cancel(true));
    }

    private synchronized void closeMembership(Object actorKey, long id) {
        Membership current = actors.get(actorKey);
        if (current != null && current.id == id) removeActorInternal(actorKey);
    }

    private synchronized void clearPending(
            Object actorKey, CompletableFuture<NavigationPlan> future) {
        pendingSearches.remove(actorKey, future);
    }

    private void removeActorInternal(Object actorKey) {
        CompletableFuture<NavigationPlan> pending =
                pendingSearches.remove(actorKey);
        if (pending != null) pending.cancel(true);
        Membership membership = actors.remove(actorKey);
        if (membership == null) return;
        membership.region.activeMembers--;
        TargetUsage usage = membership.region.targets.get(membership.target);
        if (usage != null) usage.members--;
    }

    private void ensureOpen() {
        if (closed) throw new IllegalStateException("coordinator is closed");
    }

    private static double graphLength(List<PathNode> nodes) {
        double length = 0;
        for (int index = 1; index < nodes.size(); index++) {
            length += nodes.get(index - 1).asVec()
                    .distance(nodes.get(index).asVec());
        }
        return length;
    }

    /**
     * {@code retainedNodes} is the waypoint-mesh endpoint count. The three
     * plan-map sizes are counted separately because a region retains routes
     * in maps the mesh does not reach: {@code observedPlans} while a region
     * is still gathering evidence, {@code certifiedPlans} once it has, and
     * {@code publishedPlans} for the compositions a follower may replay.
     */
    private static final class Region {
        final RegionKey key;
        final RouteGraph graph = new RouteGraph();
        final Map<ObservationKey, NavigationPlan> observedPlans = new HashMap<>();
        final Map<ObservationKey, NavigationPlan> certifiedPlans = new HashMap<>();
        // Access ordered so eviction can start at the coldest destination.
        final LinkedHashMap<TargetNode, TargetUsage> targets =
                new LinkedHashMap<>(16, 0.75f, true);
        final Map<TargetNode, SharedRouteMesh.TargetField<MeshNode>> fields =
                new HashMap<>();
        final Map<ObservationKey, Published> publishedPlans =
                new HashMap<>();
        Set<EdgeKey> fieldEdges = Set.of();
        long topologyVersion;
        long worldRevision;
        long lastRequestTick;
        int requestCount;
        int activeMembers;
        boolean promoted;
        boolean promoting;
        boolean rebuilding;

        Region(RegionKey key, long worldRevision, long tick) {
            this.key = key;
            this.worldRevision = worldRevision;
            lastRequestTick = tick;
        }

        /**
         * Whether promotion has been decided. A region only becomes
         * {@link #promoted} when the rebuild that promotion started publishes
         * the mesh it promises, so no observer can see a region that offers a
         * shared field it cannot yet serve.
         */
        boolean certifying() {
            return promoted || promoting;
        }

        void reset(long revision, long tick) {
            worldRevision = revision;
            lastRequestTick = tick;
            requestCount = activeMembers;
            promoted = false;
            promoting = false;
            graph.edges.clear();
            observedPlans.clear();
            certifiedPlans.clear();
            fields.clear();
            fieldEdges = Set.of();
            publishedPlans.clear();
            invalidate();
        }

        Optional<NavigationPlan> sharedPlan(
                SourceNode source, TargetNode target) {
            if (!promoted) return Optional.empty();
            ObservationKey key = new ObservationKey(source, target);
            NavigationPlan certificate = certifiedPlans.get(key);
            if (certificate == null) return Optional.empty();
            Published published = publishedPlans.get(key);
            // A composition answers a request only while it is still the one
            // proven identical to this key's live A* certificate, so a result
            // published from an older graph can never outlive its proof.
            return published == null || published.certificate() != certificate
                    ? Optional.empty() : Optional.of(published.plan());
        }

        void rebuild(SharedMeshPolicy config) {
            RebuildSnapshot snapshot = snapshot();
            publish(snapshot, computeRebuild(snapshot, config));
        }

        RebuildSnapshot snapshot() {
            return new RebuildSnapshot(
                    topologyVersion, worldRevision, certifying(),
                    Map.copyOf(graph.edges), List.copyOf(targets.keySet()),
                    Map.copyOf(certifiedPlans), Map.copyOf(fields),
                    fieldEdges, Map.copyOf(publishedPlans));
        }

        void publish(RebuildSnapshot snapshot, RebuildResult result) {
            if (topologyVersion != snapshot.version()
                    || worldRevision != snapshot.worldRevision()
                    || certifying() != snapshot.promoted()) return;
            promoted = snapshot.promoted();
            promoting = false;
            fields.clear();
            fields.putAll(result.fields());
            fieldEdges = result.fieldEdges();
            publishedPlans.clear();
            publishedPlans.putAll(result.published());
        }

        /** Retires every rebuild snapshot taken before this topology change. */
        void invalidate() {
            topologyVersion++;
        }

        int nodeCount() {
            return graph.nodeCount();
        }

        boolean canRetain(SourceNode source, NavigationPlan plan,
                          TargetNode target, int maximumNodes) {
            return graph.canRetain(source, plan, target, maximumNodes);
        }

        /**
         * Retains one certified route and reports whether the region moved.
         * A repeat that adds no edge is dropped rather than installed:
         * swapping in an indistinguishable certificate would retire the
         * composition published against it and leave the key unanswerable
         * until the next rebuild landed.
         */
        boolean certifyAndRetain(
                SourceNode source, NavigationPlan plan, TargetNode target) {
            ObservationKey key = new ObservationKey(source, target);
            NavigationPlan held = certifiedPlans.get(key);
            boolean added = graph.retainPath(source, plan, target);
            if (!added && held != null && repeats(held, plan)) return false;
            certifiedPlans.put(key, plan);
            invalidate();
            return true;
        }

        /** Two certificates neither the mesh nor a follower can tell apart. */
        private static boolean repeats(
                NavigationPlan held, NavigationPlan plan) {
            return NavigationPlanParity.semanticallyEquivalent(held, plan)
                    && held.nodeCosts().equals(plan.nodeCosts());
        }

        void observe(
                SourceNode source, TargetNode target, NavigationPlan plan,
                SharedMeshPolicy config) {
            if (plan.nodes().size() > config.maximumNodesPerRegion()) return;
            ObservationKey key = new ObservationKey(source, target);
            if (observedPlans.containsKey(key)
                    || observedPlans.size() < config.promotionRequests()) {
                observedPlans.put(key, plan);
                certifiedPlans.put(key, plan);
                invalidate();
            }
        }

        void promoteObserved(SharedMeshPolicy config) {
            for (Map.Entry<ObservationKey, NavigationPlan> entry
                    : observedPlans.entrySet()) {
                SourceNode source = entry.getKey().source;
                TargetNode target = entry.getKey().target;
                NavigationPlan plan = entry.getValue();
                if (canRetain(source, plan, target,
                        config.maximumNodesPerRegion())) {
                    graph.retainPath(source, plan, target);
                }
            }
            observedPlans.clear();
            invalidate();
        }

        /**
         * Drops one destination and everything that answers it. The retained
         * waypoint skeleton is collision-validated for this world revision and
         * is destination independent, so it survives a target that moved on.
         */
        boolean forget(TargetNode target) {
            boolean changed = targets.remove(target) != null;
            changed |= observedPlans.keySet().removeIf(
                    observation -> observation.target.equals(target));
            changed |= certifiedPlans.keySet().removeIf(
                    observation -> observation.target.equals(target));
            changed |= publishedPlans.keySet().removeIf(
                    observation -> observation.target.equals(target));
            changed |= fields.remove(target) != null;
            changed |= graph.removeEdgesInto(target);
            if (changed) invalidate();
            return changed;
        }
    }

    /** One composition and the individual A* certificate that proved it. */
    private record Published(
            NavigationPlan certificate, NavigationPlan plan) {
    }

    /**
     * Everything a rebuild reads, copied under the monitor so the reverse
     * Dijkstra it drives can run without holding one. The carried target
     * fields are the shared immutable ones a previous rebuild published.
     */
    private record RebuildSnapshot(
            long version, long worldRevision, boolean promoted,
            Map<EdgeKey, NavigationPlan> edges, List<TargetNode> targets,
            Map<ObservationKey, NavigationPlan> certified,
            Map<TargetNode, SharedRouteMesh.TargetField<MeshNode>> fields,
            Set<EdgeKey> fieldEdges,
            Map<ObservationKey, Published> published) {
    }

    private record RebuildResult(
            Map<TargetNode, SharedRouteMesh.TargetField<MeshNode>> fields,
            Set<EdgeKey> fieldEdges,
            Map<ObservationKey, Published> published) {
    }

    /**
     * The retained directed edge set of one region. Like its owning region it
     * is unsynchronized and is only ever touched under the coordinator monitor.
     */
    private static final class RouteGraph {
        final Map<EdgeKey, NavigationPlan> edges = new HashMap<>();

        int nodeCount() {
            return endpoints().size();
        }

        /**
         * The answering source is passed in rather than rebuilt from the
         * plan: a plan carries its start position but not the traversal
         * snapshot that resolves it, so deriving one here would count two
         * distinct mesh sources as one.
         */
        boolean canRetain(SourceNode source, NavigationPlan plan,
                          TargetNode target, int maximumNodes) {
            Set<MeshNode> nodes = endpoints();
            for (PathNode node : plan.nodes()) {
                nodes.add(new WaypointNode(WaypointIdentity.of(node)));
            }
            nodes.add(source);
            nodes.add(target);
            return nodes.size() <= maximumNodes;
        }

        /** Adds every missing segment and reports whether any was new. */
        boolean retainPath(
                SourceNode source, NavigationPlan plan, TargetNode target) {
            List<PathNode> nodes = plan.nodes();
            if (nodes.isEmpty()) return false;
            boolean added = false;
            PathNode firstNode = nodes.getFirst();
            WaypointNode firstWaypoint = new WaypointNode(
                    WaypointIdentity.of(firstNode));
            NavigationPlan connector = new NavigationPlan(
                    plan.start(), firstNode.asVec(), plan.boundingBox(),
                    plan.profile(), PathStatus.FOUND,
                    List.of(firstNode), 0, 0, costs(plan, 0, 1));
            added |= edges.putIfAbsent(
                    new EdgeKey(source, firstWaypoint), connector) == null;
            for (int index = 0; index < nodes.size() - 1; index++) {
                PathNode first = nodes.get(index);
                PathNode second = nodes.get(index + 1);
                WaypointNode from = new WaypointNode(
                        WaypointIdentity.of(first));
                MeshNode to = index == nodes.size() - 2
                        ? target
                        : new WaypointNode(WaypointIdentity.of(second));
                Point segmentTarget = to == target
                        ? plan.target() : second.asVec();
                NavigationPlan segment = new NavigationPlan(
                        first.asVec(), segmentTarget, plan.boundingBox(),
                        plan.profile(), to == target
                                ? plan.status() : PathStatus.FOUND,
                        List.of(first, second), 0, 0,
                        costs(plan, index, index + 2));
                added |= edges.putIfAbsent(
                        new EdgeKey(from, to), segment) == null;
            }
            if (nodes.size() == 1) {
                NavigationPlan terminal = new NavigationPlan(
                        firstNode.asVec(), plan.target(), plan.boundingBox(),
                        plan.profile(), plan.status(),
                        List.of(firstNode), 0, 0, costs(plan, 0, 1));
                added |= edges.putIfAbsent(
                        new EdgeKey(firstWaypoint, target), terminal) == null;
            }
            return added;
        }

        /** The charged costs of one decomposed span, aligned with its nodes. */
        private static List<PathNodeCost> costs(
                NavigationPlan plan, int from, int to) {
            return plan.nodeCosts().isEmpty()
                    ? List.of() : plan.nodeCosts().subList(from, to);
        }

        boolean removeEdgesInto(TargetNode target) {
            return edges.keySet().removeIf(edge -> edge.to.equals(target));
        }

        private Set<MeshNode> endpoints() {
            Set<MeshNode> nodes = new HashSet<>();
            edges.keySet().forEach(edge -> {
                nodes.add(edge.from);
                nodes.add(edge.to);
            });
            return nodes;
        }
    }

    private static final class TargetUsage {
        int members;
        long lastUsedTick;

        TargetUsage(long tick) {
            lastUsedTick = tick;
        }
    }

    private record Membership(long id, Region region, TargetNode target) {
    }

    private sealed interface MeshNode permits SourceNode, WaypointNode,
            TargetNode {
    }

    private record SourceNode(
            ExactPosition position, StartCompatibility start)
            implements MeshNode {
    }

    private record WaypointNode(WaypointIdentity waypoint)
            implements MeshNode {
    }

    private record TargetNode(Object targetKey, TargetCell cell)
            implements MeshNode {
    }

    private record EdgeKey(MeshNode from, MeshNode to) {
    }

    private record ObservationKey(SourceNode source, TargetNode target) {
    }

    private record ExactPosition(long x, long y, long z) {
        static ExactPosition of(Point point) {
            return new ExactPosition(
                    Double.doubleToLongBits(point.x()),
                    Double.doubleToLongBits(point.y()),
                    Double.doubleToLongBits(point.z()));
        }
    }

    private record WaypointIdentity(
            long x, long y, long z,
            int graphX, int graphY, int graphZ,
            PathNode.Movement movement) {
        static WaypointIdentity of(PathNode node) {
            return new WaypointIdentity(
                    Double.doubleToLongBits(node.x()),
                    Double.doubleToLongBits(node.y()),
                    Double.doubleToLongBits(node.z()),
                    node.graphX(), node.graphY(), node.graphZ(),
                    node.movement());
        }
    }

    private record RegionKey(
            Identity world, int regionX, int regionY, int regionZ,
            Compatibility compatibility) {
        static RegionKey of(
                NavigationRequest request, SharedMeshPolicy config) {
            return new RegionKey(
                    new Identity(request.blocks()),
                    Math.floorDiv(request.start().blockX(), config.regionSize()),
                    Math.floorDiv(request.start().blockY(),
                            config.verticalRegionSize()),
                    Math.floorDiv(request.start().blockZ(), config.regionSize()),
                    Compatibility.of(request));
        }
    }

    /**
     * The whole bounding box is part of regional compatibility. Evaluators
     * read its anchor as well as its extent, so two footprints that share
     * width, height, and depth can still search different graphs.
     *
     * <p>Only the build-height half of the live traversal snapshot belongs
     * here: both bounds change the graph beyond the start node, by moving the
     * downward scans that classify an arbitrary neighbour and the destination
     * a ground target resolves to, so they must keep regions apart. Everything
     * else in the snapshot keys the source instead; see
     * {@link StartCompatibility}.</p>
     */
    private record Compatibility(
            BoundingBox boundingBox,
            NavigationProfile profile, List<NavigationInfluence> influences,
            int seaLevel, int minBuildHeight, int maxBuildHeight,
            double maxPathLength, int reachRange,
            double maxVisitedMultiplier, double nodeSearchRange) {
        static Compatibility of(NavigationRequest request) {
            EntityTraversalState state = request.entityState();
            return new Compatibility(
                    request.boundingBox(), request.profile(),
                    NavigationInfluence.byDeclaredEquality(
                            request.influences()), request.seaLevel(),
                    state.minBuildHeight(), state.maxBuildHeight(),
                    request.maxPathLength(),
                    request.reachRange(), request.maxVisitedMultiplier(),
                    request.nodeSearchRange());
        }
    }

    /**
     * The half of the live traversal snapshot that resolves only the search's
     * start node, and therefore keys a source rather than a region.
     *
     * <p>These fields are read only while resolving the start node. None
     * reaches neighbour expansion, terrain classification, or destination
     * normalization, so two requests differing only here search the same graph
     * and may share one region, one retained mesh, and one target field.
     *
     * <p>They are keyed rather than dropped because each can send one exact
     * position to a different start node, and the whole search hangs off that
     * node. The parity gate cannot catch it:
     * {@link NavigationPlanParity#requestCompatible} compares the raw request
     * start, which is bit-identical across such a toggle, and a {@link
     * NavigationPlan} carries no traversal snapshot at all. This is the whole
     * reason a jumping mob can never be answered from a grounded mob's
     * certificate.</p>
     */
    private record StartCompatibility(
            boolean onGround, boolean inWater, Set<Block> standableFluids,
            List<Point> startCandidates) {
        static StartCompatibility of(EntityTraversalState state) {
            return new StartCompatibility(
                    state.onGround(), state.inWater(),
                    state.standableFluids(),
                    state.pathfindingStartCandidates());
        }
    }

    private static final class Identity {
        final Object value;

        Identity(Object value) {
            this.value = Objects.requireNonNull(value);
        }

        @Override
        public boolean equals(Object object) {
            return object instanceof Identity identity
                    && value == identity.value;
        }

        @Override
        public int hashCode() {
            return System.identityHashCode(value);
        }
    }
}
