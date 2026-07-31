package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.metrics.NavigationMetrics;import ca.atlasengine.pathfinding.metrics.NavigationMetricsSnapshot;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.adaptive.SharedMeshNavigation;
import ca.atlasengine.pathfinding.adaptive.SharedMeshOptions;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import net.minestom.server.coordinate.Point;
import net.minestom.server.entity.Entity;

import net.minestom.server.event.EventListener;
import net.minestom.server.event.instance.InstanceBlockUpdateEvent;
import net.minestom.server.instance.Instance;
import ca.atlasengine.pathfinding.event.NavigationEvent;
import ca.atlasengine.pathfinding.event.NavigationEventSink;
import net.minestom.server.event.EventFilter;
import net.minestom.server.event.EventNode;
import java.util.Collections;
import java.util.WeakHashMap;
import java.util.List;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.random.RandomGenerator;

/**
 * Primary owned entry point for asynchronous entity navigation.
 *
 * <p>Create one system per server or instance group, create a controller for
 * each navigating entity, and call that controller's {@link
 * EntityNavigationController#tick()} from the entity/instance tick thread.
 * Closing the system cancels outstanding searches and terminates its bounded
 * worker pool.</p>
 *
 * <p>Every method here plans one individual A* search per request. Shared
 * regional planning is a separate, explicitly enabled subsystem reached
 * through {@link #sharedMesh()}; see {@link
 * Builder#sharedMesh(SharedMeshOptions)}.</p>
 */
public final class NavigationSystem implements AutoCloseable {
    private final AsyncEntityPathfindingService service;
    private final ControllerPathScheduler controllerScheduler;
    private final ControllerSchedulingOptions controllerOptions;
    private final SharedMeshNavigation sharedMesh;
    private volatile EventNode<NavigationEvent> events;
    private final NavigationEventSink sink = new NavigationEventSink() {
        @Override
        public boolean listening(Class<? extends NavigationEvent> type) {
            EventNode<NavigationEvent> node = events;
            return node != null && node.hasListener(type);
        }

        @Override
        public void emit(NavigationEvent event) {
            EventNode<NavigationEvent> node = events;
            if (node != null) node.call(event);
        }
    };
    /**
     * Controllers this system created, held weakly: a caller that forgets
     * close() leaks nothing, and a collected controller stops being
     * notified.
     */
    private final Set<EntityNavigationController> controllers =
            Collections.synchronizedSet(
                    Collections.newSetFromMap(new WeakHashMap<>()));
    private final NavigationMetrics metrics;
    private final double movementPerTick;

    private NavigationSystem(Builder builder) {
        metrics = builder.metrics == null
                ? new NavigationMetrics() : builder.metrics;
        service = new AsyncEntityPathfindingService(
                builder.parallelism, builder.queueCapacity, metrics);
        controllerOptions = new ControllerSchedulingOptions(
                builder.maximumDeferredControllerRequests,
                Math.min(builder.maximumConcurrentControllerSearches,
                        builder.parallelism),
                builder.minimumSearchStallTicks,
                builder.shedBackoffTicks);
        controllerScheduler = new ControllerPathScheduler(
                service, controllerOptions);
        sharedMesh = new SharedMeshNavigation(this, builder.sharedMesh);
        movementPerTick = builder.movementPerTick;
    }

    public static NavigationSystem create() {
        return builder().build();
    }

    public static Builder builder() {
        return new Builder();
    }

    /** Creates a baseline controller using this system's default speed. */
    public EntityNavigationController controller(Entity entity) {
        return controller(entity, movementPerTick);
    }

    /** Creates a baseline controller with an explicit movement speed. */
    public EntityNavigationController controller(
            Entity entity, double movementPerTick) {
        validate(entity, movementPerTick);
        return track(EntityNavigationController.builtin(
                entity, service, movementPerTick,
                controllerScheduler::submitLatest, controllerOptions));
    }

    /**
     * Creates a controller with an explicit traversal profile. This is the integration point
     * for model-driven or proxy entities whose Minestom carrier type does not describe how the
     * visible mob should navigate (for example an invisible parrot carrying a ground model).
     */
    public EntityNavigationController controller(
            Entity entity, NavigationProfile profile, double movementPerTick) {
        validate(entity, movementPerTick);
        if (profile == null) {
            throw new IllegalArgumentException("invalid controller arguments");
        }
        return track(new EntityNavigationController(
                entity, service, profile, movementPerTick,
                MovementExecutionMode.PHYSICS_VELOCITY,
                controllerScheduler::submitLatest, controllerOptions));
    }

    /**
     * Creates a baseline controller using the mob AI's authoritative random
     * stream for randomized start sampling.
     */
    public EntityNavigationController controller(
            Entity entity, RandomGenerator random) {
        validate(entity, movementPerTick);
        if (random == null) throw new IllegalArgumentException("random");
        return track(EntityNavigationController.builtin(
                entity, service, movementPerTick, random,
                controllerScheduler::submitLatest, controllerOptions));
    }

    private static void validate(Entity entity, double movementPerTick) {
        if (entity == null || !Double.isFinite(movementPerTick)
                || movementPerTick <= 0) {
            throw new IllegalArgumentException("invalid controller arguments");
        }
    }

    public CompletableFuture<PathResult> submit(NavigationRequest request) {
        return service.submit(request);
    }

    /** Computes an immutable plan that can be cached or fed to a follower. */
    public CompletableFuture<NavigationPlan> plan(NavigationRequest request) {
        if (request == null) throw new IllegalArgumentException("request");
        return service.submit(request).thenApply(result ->
                NavigationPlan.from(request, result));
    }

    /** Advanced coalescing request API, normally keyed by entity UUID. */
    public CompletableFuture<PathResult> submitLatest(
            Object key, NavigationRequest request) {
        return service.submitLatest(key, request);
    }

    /** Coalescing variant of {@link #plan(NavigationRequest)}. */
    public CompletableFuture<NavigationPlan> planLatest(
            Object key, NavigationRequest request) {
        if (request == null) throw new IllegalArgumentException("request");
        return controllerScheduler.submitLatest(key, request, 10).thenApply(result ->
                NavigationPlan.from(request, result));
    }

    /**
     * The shared-mesh planner of this system. It is present on every system
     * but plans individually unless {@link
     * Builder#sharedMesh(SharedMeshOptions)} opted in, and can be switched off
     * again at runtime.
     */
    public SharedMeshNavigation sharedMesh() {
        return sharedMesh;
    }

    public int activeSearches() {
        return service.activeCount();
    }

    public int queuedSearches() {
        return service.queuedCount();
    }

    /** Controller searches admitted to the worker layer. */
    public int activeControllerSearches() {
        return controllerScheduler.runningCount();
    }

    /** Latest controller intents waiting outside the executor queue. */
    public int deferredControllerSearches() {
        return controllerScheduler.deferredCount();
    }

    private EntityNavigationController track(
            EntityNavigationController controller) {
        controllers.add(controller);
        if (events != null) controller.eventSink(sink);
        return controller;
    }

    /**
     * Navigation events from every controller this system owns.
     *
     * <p>The node is created the first time it is asked for, and controllers
     * only build an event when this node has a listener for its type,
     * so a system nobody listens to allocates nothing on the navigation path.
     * Events are called on the thread that ticked the controller.</p>
     *
     * <pre>{@code
     * navigation.eventNode().addListener(NavigationStuckEvent.class,
     *         event -> unwedge(event.getEntity()));
     * }</pre>
     */
    public synchronized EventNode<NavigationEvent> eventNode() {
        if (events == null) {
            events = EventNode.type("navigation-" + hashCode(),
                    EventFilter.from(NavigationEvent.class, null, null));
            List<EntityNavigationController> existing;
            synchronized (controllers) {
                existing = List.copyOf(controllers);
            }
            for (EntityNavigationController controller : existing) {
                controller.eventSink(sink);
            }
        }
        return events;
    }

    /**
     * Replans any route this system owns that the changed block may cross.
     *
     * <p>A controller decides for itself whether the block is near enough to
     * the stretch it has left to walk, so calling this for every block change
     * is cheap and safe.</p>
     */
    public void notifyBlockChanged(Point changedBlock) {
        if (changedBlock == null) return;
        List<EntityNavigationController> snapshot;
        synchronized (controllers) {
            snapshot = List.copyOf(controllers);
        }
        for (EntityNavigationController controller : snapshot) {
            controller.onBlockChanged(changedBlock);
        }
    }

    /**
     * Subscribes to {@code instance} so block changes replan affected routes
     * without the caller wiring anything.
     *
     * <p>Listeners run on whichever thread applied the block change, which for
     * ordinary instance edits is the tick thread that also ticks the
     * controllers. Returns the registered listener so it can be removed.</p>
     */
    public EventListener<InstanceBlockUpdateEvent> watchBlockChanges(
            Instance instance) {
        if (instance == null) throw new IllegalArgumentException("instance");
        EventListener<InstanceBlockUpdateEvent> listener =
                EventListener.of(InstanceBlockUpdateEvent.class,
                        event -> notifyBlockChanged(event.getBlockPosition()));
        instance.eventNode().addListener(listener);
        return listener;
    }

    /**
     * Everything this system was configured with, in one value.
     *
     * <p>Reading configuration back one getter at a time made the settings
     * look unrelated and let {@code queueCapacity} and {@code parallelism}
     * drift from the identical pair already on
     * {@link NavigationMetricsSnapshot#queue()}.</p>
     */
    public NavigationOptions options() {
        return new NavigationOptions(
                service.parallelism(),
                service.queueCapacity(),
                controllerOptions.maximumConcurrentSearches(),
                controllerOptions.maximumDeferredRequests(),
                controllerOptions.minimumSearchStallTicks(),
                controllerOptions.shedBackoffTicks());
    }

    /**
     * Cumulative counters, latency histograms, adaptive reuse breakdown, and
     * controller outcomes shared by this system's service, coordinator, and
     * every controller it creates.
     */
    public NavigationMetrics metrics() {
        return metrics;
    }

    /** One immutable reading of {@link #metrics()}. */
    public NavigationMetricsSnapshot metricsSnapshot() {
        return metrics.snapshot();
    }

    @Override
    public void close() {
        // The worker pool must be shut down even if coordinator teardown
        // fails, or a failure there would leak the whole pool.
        try {
            sharedMesh.close();
        } finally {
            try {
                controllerScheduler.close();
            } finally {
                service.close();
            }
        }
    }

    public static final class Builder {
        private int parallelism = Math.max(1, Math.min(8,
                Runtime.getRuntime().availableProcessors() / 2));
        private int queueCapacity = 256;
        private int maximumDeferredControllerRequests = 16_384;
        private int maximumConcurrentControllerSearches = Integer.MAX_VALUE;
        private int minimumSearchStallTicks = 60;
        private int shedBackoffTicks = 20;
        private double movementPerTick = 0.2;
        private SharedMeshOptions sharedMesh = SharedMeshOptions.DISABLED;
        private NavigationMetrics metrics;

        private Builder() {
        }

        public Builder parallelism(int value) {
            if (value <= 0) throw new IllegalArgumentException("parallelism");
            parallelism = value;
            return this;
        }

        public Builder queueCapacity(int value) {
            if (value <= 0) throw new IllegalArgumentException("queueCapacity");
            queueCapacity = value;
            return this;
        }

        /** Maximum controller replans retained before lower-priority work sheds. */
        public Builder maximumDeferredControllerRequests(int value) {
            if (value <= 0) throw new IllegalArgumentException(
                    "maximumDeferredControllerRequests");
            maximumDeferredControllerRequests = value;
            return this;
        }

        /** Controller searches admitted concurrently; worker parallelism remains the hard cap. */
        public Builder maximumConcurrentControllerSearches(int value) {
            if (value <= 0) throw new IllegalArgumentException(
                    "maximumConcurrentControllerSearches");
            maximumConcurrentControllerSearches = value;
            return this;
        }

        /** Minimum controller wait before an apparently lost search is replaced. */
        public Builder minimumSearchStallTicks(int value) {
            if (value <= 0) throw new IllegalArgumentException(
                    "minimumSearchStallTicks");
            minimumSearchStallTicks = value;
            return this;
        }

        /** Retry delay after the bounded execution layer sheds an admitted search. */
        public Builder shedBackoffTicks(int value) {
            if (value <= 0) throw new IllegalArgumentException("shedBackoffTicks");
            shedBackoffTicks = value;
            return this;
        }

        public Builder movementPerTick(double value) {
            if (!Double.isFinite(value) || value <= 0) {
                throw new IllegalArgumentException("movementPerTick");
            }
            movementPerTick = value;
            return this;
        }

        /**
         * Opts this system in to shared-mesh planning. Without this call the
         * system plans every request individually and {@link
         * NavigationSystem#sharedMesh()} reports {@code enabled() == false}.
         * Read {@link SharedMeshOptions} before enabling it: what a region
         * partitions on decides whether the mesh saves work or only costs it.
         */
        public Builder sharedMesh(SharedMeshOptions value) {
            if (value == null) throw new IllegalArgumentException("sharedMesh");
            sharedMesh = value;
            return this;
        }

        /** Aggregates this system's telemetry into a shared recorder. */
        public Builder metrics(NavigationMetrics value) {
            if (value == null) throw new IllegalArgumentException("metrics");
            metrics = value;
            return this;
        }

        public NavigationSystem build() {
            return new NavigationSystem(this);
        }
    }
}
