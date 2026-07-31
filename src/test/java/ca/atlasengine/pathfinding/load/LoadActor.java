package ca.atlasengine.pathfinding.load;

import ca.atlasengine.pathfinding.metrics.NavigationMetrics;import net.minestom.server.instance.block.Block;
import ca.atlasengine.pathfinding.adaptive.SharedMeshRequest;
import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.EntityNavigationController;
import ca.atlasengine.pathfinding.search.EntityTraversalState;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.NavigationState;
import ca.atlasengine.pathfinding.NavigationSystem;
import ca.atlasengine.pathfinding.result.PathStatus;
import ca.atlasengine.pathfinding.adaptive.SharedMeshHandle;
import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.NavigationPlanCache;
import ca.atlasengine.pathfinding.internal.adaptive.TargetCell;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.LivingEntity;
import net.minestom.server.entity.attribute.Attribute;
import net.minestom.server.instance.Instance;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.CancellationException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionException;
import java.util.function.Supplier;

/** One navigating mob: adaptive planning plus the real follow controller. */
final class LoadActor implements AutoCloseable {
    enum Shape {
        /** Fixed endpoints, revisited forever. */
        PATROL,
        /** Many mobs onto one shared objective. */
        CONVERGE,
        /** A continuously moving target entity. */
        PURSUIT
    }

    record CacheKey(UUID actor, TargetCell cell) {
    }

    /** Matches the follower's own arrival tolerance. */
    private static final double GOAL_TOLERANCE = 2.0;

    private final NavigationSystem system;
    private final LoadMetrics metrics;
    private final NavigationPlanCache<CacheKey> cache;
    private final EntityCreature entity;
    private final EntityNavigationController controller;
    private final Shape shape;
    private final Pos[] waypoints;
    private final Object[] waypointKeys;
    private final Supplier<Entity> pursued;
    private final int replanTicks;
    private final int jitter;
    private final double range;

    private SharedMeshHandle handle;
    private boolean attempted;
    private boolean episodeCounted;
    private long lastSubmitTick = Long.MIN_VALUE / 4;
    private int leg;
    private TargetCell lastRequestedCell;
    private double submitDistance;
    private double travelled;
    private Pos previousPosition;
    private boolean closed;

    LoadActor(NavigationSystem system, LoadMetrics metrics,
              NavigationPlanCache<CacheKey> cache, EntityCreature entity,
              Shape shape, Pos[] waypoints, Object[] waypointKeys,
              Supplier<Entity> pursued, int replanTicks, int jitter) {
        this.system = system;
        this.metrics = metrics;
        this.cache = cache;
        this.entity = entity;
        this.shape = shape;
        this.waypoints = waypoints;
        this.waypointKeys = waypointKeys;
        this.pursued = pursued;
        this.replanTicks = replanTicks;
        this.jitter = jitter;
        this.controller = system.controller(entity);
        double follow = entity instanceof LivingEntity living
                ? living.getAttribute(Attribute.FOLLOW_RANGE).getValue() : 16;
        this.range = Math.max(follow,
                BuiltinNavigationProfiles.requiredPathLength(entity));
    }

    EntityCreature entity() {
        return entity;
    }

    Shape shape() {
        return shape;
    }

    double travelled() {
        return travelled;
    }

    NavigationState state() {
        return controller.state();
    }

    void tick(long tick, long worldRevision) {
        if (closed) return;
        Pos position = entity.getPosition();
        if (previousPosition != null) {
            travelled += position.distance(previousPosition);
        }
        previousPosition = position;
        countTerminal();
        // Consume a landed plan before considering a replan, so a route is
        // never discarded on the tick it arrives.
        if (handle != null && !attempted && handle.plan().isDone()) {
            apply(worldRevision);
        }
        if (shouldSubmit(tick)) submit(tick, worldRevision);
        controller.tick();
    }

    private void countTerminal() {
        NavigationState state = controller.state();
        if (episodeCounted || !terminal(state)) return;
        episodeCounted = true;
        // A mob already standing on its goal completes instantly; only
        // episodes with ground to cover say anything about progress.
        if (submitDistance > GOAL_TOLERANCE) metrics.travelTerminal(state);
        if (shape == Shape.PATROL && state == NavigationState.COMPLETED) {
            leg = (leg + 1) % waypoints.length;
        }
    }

    private static boolean terminal(NavigationState state) {
        return state == NavigationState.COMPLETED
                || state == NavigationState.FAILED
                || state == NavigationState.CANCELLED
                || state == NavigationState.STUCK;
    }

    /**
     * A periodic replan that cancelled the search it is waiting for would
     * resubmit forever and the mob would never follow a route at all. That was
     * certain while the loop that drives this actor free-ran, where an
     * iteration cost a couple of milliseconds on a warm JVM and a search cost
     * tens; {@link LoadClock} now holds an iteration to the fifty milliseconds
     * a cadence expressed in ticks assumes, and the rule stays because a
     * loaded pool still answers later than the tick that asked. Only a target
     * that actually moved may supersede work already in flight.
     */
    private boolean shouldSubmit(long tick) {
        long elapsed = tick - lastSubmitTick;
        if (handle == null && !attempted) return true;
        if (elapsed < replanTicks / 2 + jitter) return false;
        boolean targetMoved = shape == Shape.PURSUIT
                && !goalCell().equals(lastRequestedCell);
        if (searchInFlight()) return targetMoved;
        if (elapsed >= replanTicks + jitter) return true;
        return episodeCounted || targetMoved;
    }

    private boolean searchInFlight() {
        return handle != null && !attempted && !handle.plan().isDone();
    }

    private Point goal() {
        if (shape == Shape.PURSUIT) {
            Entity target = pursued.get();
            return target == null ? waypoints[0] : target.getPosition();
        }
        return waypoints[leg];
    }

    private TargetCell goalCell() {
        return TargetCell.of(goal());
    }

    private Object targetKey() {
        if (shape == Shape.PURSUIT) {
            Entity target = pursued.get();
            return target == null ? waypointKeys[0] : target.getUuid();
        }
        return waypointKeys[leg];
    }

    private void submit(long tick, long worldRevision) {
        Instance instance = entity.getInstance();
        if (instance == null) return;
        if (handle != null) handle.close();
        handle = null;
        attempted = false;
        episodeCounted = false;
        lastSubmitTick = tick;
        Point destination = goal();
        submitDistance = entity.getPosition().distance(destination);
        lastRequestedCell = TargetCell.of(destination);
        if (shape == Shape.PATROL && replayFromCache(worldRevision)) return;
        NavigationRequest request = request(instance, destination);
        long started = System.nanoTime();
        handle = system.sharedMesh().plan(SharedMeshRequest.builder(request).actor(entity.getUuid()).target(targetKey()).worldRevision(worldRevision).currentTick(tick).build());
        metrics.dispatchTook(System.nanoTime() - started);
        metrics.adaptiveCall();
        // Latency and outcome counts come from NavigationMetrics. The harness
        // still classifies its own handle futures, because only the caller
        // knows which failure modes its contract permits.
        handle.plan().whenComplete((plan, failure) -> {
            if (failure == null) return;
            Throwable cause = failure instanceof CompletionException
                    ? failure.getCause() : failure;
            // Overload is a value now, so cancellation is the only failure
            // this contract permits.
            if (cause instanceof CancellationException) {
                metrics.handleCancelled();
            } else metrics.unexpected(cause);
        });
    }

    private boolean replayFromCache(long worldRevision) {
        Optional<NavigationPlan> cached = cache.get(
                new CacheKey(entity.getUuid(), lastRequestedCell),
                worldRevision, entity.getPosition(), 1.0);
        if (cached.isEmpty()) return false;
        try {
            controller.follow(cached.orElseThrow());
        } catch (IllegalArgumentException stale) {
            metrics.stalePlan();
            return false;
        }
        metrics.cacheHit();
        attempted = true;
        return true;
    }

    private NavigationRequest request(Instance instance, Point destination) {
        NavigationProfile profile = BuiltinNavigationProfiles.forEntity(entity);
        boolean inWater = BlockTraversalData.hasWaterFluid(
                instance.getBlock(entity.getPosition()));
        Set<Block> standable = profile.mobProfile().standsOnLava()
                ? Set.of(Block.LAVA) : Set.of();
        EntityTraversalState state = new EntityTraversalState(
                entity.isOnGround(), inWater, standable,
                instance.getCachedDimensionType().minY(),
                instance.getCachedDimensionType().maxY(), List.of());
        return NavigationRequest.builder(instance, entity.getPosition(),
                        destination, entity.getBoundingBox(), profile)
                .maxPathLength(range).nodeSearchRange(range)
                .entityState(state).build();
    }

    private void apply(long worldRevision) {
        attempted = true;
        CompletableFuture<NavigationPlan> future = handle.plan();
        NavigationPlan plan;
        try {
            plan = future.join();
        } catch (CancellationException | CompletionException failure) {
            return;
        }
        if (plan != null && plan.status() == PathStatus.SHED) {
            metrics.handleShed();
            return;
        }
        if (plan == null || !plan.usable()) {
            metrics.unusablePlan();
            return;
        }
        try {
            controller.follow(plan);
        } catch (IllegalArgumentException stale) {
            metrics.stalePlan();
            return;
        }
        if (shape == Shape.PATROL) {
            cache.put(new CacheKey(entity.getUuid(), lastRequestedCell),
                    worldRevision, plan);
            metrics.cachePut();
        }
    }

    @Override
    public void close() {
        if (closed) return;
        closed = true;
        if (handle != null) handle.close();
        handle = null;
        controller.close();
        system.sharedMesh().forgetActor(entity.getUuid());
        entity.remove();
    }
}
