package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.metrics.NavigationMetrics;import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.profile.NavigationModifiers;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.result.PathStop;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.event.NavigationCancelledEvent;
import ca.atlasengine.pathfinding.event.NavigationCompletedEvent;
import ca.atlasengine.pathfinding.event.NavigationEvent;
import ca.atlasengine.pathfinding.event.NavigationEventSink;
import ca.atlasengine.pathfinding.event.NavigationFailedEvent;
import ca.atlasengine.pathfinding.event.NavigationStuckEvent;
import ca.atlasengine.pathfinding.event.PathComputedEvent;
import ca.atlasengine.pathfinding.event.PathShedEvent;
import ca.atlasengine.pathfinding.event.RouteReplanEvent;
import ca.atlasengine.pathfinding.event.TargetUnreachableEvent;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.internal.movement.BlockManipulator;
import ca.atlasengine.pathfinding.internal.movement.ClimbMovementExecutor;
import ca.atlasengine.pathfinding.internal.movement.MovementContext;
import ca.atlasengine.pathfinding.internal.movement.MovementExecutor;
import ca.atlasengine.pathfinding.internal.movement.NavigationRequestFactory;
import ca.atlasengine.pathfinding.internal.movement.PartialRouteReplanner;
import ca.atlasengine.pathfinding.internal.movement.PathPostProcessor;
import ca.atlasengine.pathfinding.internal.movement.PathSubmitter;
import ca.atlasengine.pathfinding.internal.movement.PlatformJumpMovementExecutor;
import ca.atlasengine.pathfinding.internal.movement.ProgressMonitor;
import ca.atlasengine.pathfinding.internal.movement.RouteGeometry;
import ca.atlasengine.pathfinding.internal.movement.RouteSplicer;
import ca.atlasengine.pathfinding.internal.movement.VolumeMovementExecutor;
import ca.atlasengine.pathfinding.internal.movement.WalkMovementExecutor;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;
import net.minestom.server.instance.Instance;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.CancellationException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionException;
import java.util.random.RandomGenerator;

/**
 * Tick-thread adapter that applies asynchronous paths to a Minestom entity.
 *
 * <p>Call {@link #tick()} from the entity/instance tick thread. Worker threads
 * only read the immutable request and never mutate the entity.</p>
 *
 * <p>This controller is externally driven: its caller ticks it. A pursuit
 * drives the follower it owns instead and never hands one out, so every
 * controller a caller holds is one it must tick.</p>
 */
public class EntityNavigationController implements AutoCloseable {
    private static final long MICROS_PER_TICK = 50_000;
    private static final WalkMovementExecutor WALK = new WalkMovementExecutor();
    private static final ClimbMovementExecutor CLIMB = new ClimbMovementExecutor();
    private static final MovementExecutor VOLUME = new VolumeMovementExecutor();
    private static final MovementExecutor PLATFORM_JUMP =
            new PlatformJumpMovementExecutor();

    private final Entity entity;
    private final PathSubmitter pathSubmitter;
    private final NavigationProfile navigationProfile;
    private NavigationProfile activeNavigationProfile;
    private final double movementPerTick;
    private final boolean wallClimberFallback;
    private final boolean refreshBuiltinProfile;
    private final MovementExecutionMode movementExecutionMode;
    private final NavigationMetrics metrics;
    private final ControllerSchedulingOptions schedulingOptions;
    private final MovementContext movementContext = new ControllerMovementContext();
    private final RouteGeometry geometry = new RouteGeometry(movementContext);
    private final PartialRouteReplanner replanner = new PartialRouteReplanner();
    private final RouteSplicer splicer = new RouteSplicer();
    private final BlockManipulator manipulator = new BlockManipulator();
    private final ProgressMonitor progress;

    private CompletableFuture<PathResult> pending;
    private List<PathNode> nodes = List.of();
    private int nodeIndex;
    private Point target;
    private NavigationState state = NavigationState.IDLE;
    private NavigationEventSink events = NavigationEventSink.NONE;
    private int tick;
    private int generation;
    private List<NavigationInfluence> activeInfluences = List.of();
    private NavigationModifiers activeModifiers = NavigationModifiers.NONE;
    private boolean gravityOverridden;
    private boolean previousNoGravity;
    private boolean pathTruncatedForSun;
    private int lastSubmitTick = -1;
    private int planningStartTick = -1;
    private int shedBackoffUntilTick;
    private boolean replanRequested;
    private boolean spliceSuppressed;
    private boolean travelRequired = true;
    private PathStop routeStop = PathStop.NOT_SEARCHED;
    private boolean targetUnreachable;
    private boolean unreachableRecorded;
    private int activePlatformJumpNode = -1;
    private double activePlatformJumpSpeed;
    private int platformJumpAlignmentNode = -1;
    private int platformJumpAlignmentTicks;
    private int lastGroundImpulseNode = -1;

    /**
     * Creates a controller backed by the modular movement-model dispatcher.
     *
     * <p>The supplied profile selects ground, water, flying, amphibious, or
     * wall-climber planning. Ground and wall-climber profiles retain normal
     * collision-aware walking, while water and flying profiles follow the
     * path's three-dimensional waypoints.</p>
     */
    public EntityNavigationController(Entity entity,
                                      AsyncEntityPathfindingService service,
                                      NavigationProfile profile,
                                      double movementPerTick) {
        this(entity, service, profile, movementPerTick,
                MovementExecutionMode.PHYSICS_VELOCITY);
    }

    public EntityNavigationController(
            Entity entity, AsyncEntityPathfindingService service,
            NavigationProfile profile, double movementPerTick,
            MovementExecutionMode movementExecutionMode) {
        this(entity, service, profile, movementPerTick, false,
                movementExecutionMode,
                NavigationRequestFactory.entityRandomOrNull(entity));
    }

    /**
     * Creates a modular controller with an authoritative tick-thread random
     * stream for small-flyer start sampling.
     *
     * <p>Pass the entity AI's own random source when the integration exposes
     * one. The normal constructors retain a stable UUID-derived fallback
     * because Minestom entities do not currently expose such a stream.</p>
     */
    public EntityNavigationController(
            Entity entity, AsyncEntityPathfindingService service,
            NavigationProfile profile, double movementPerTick,
            MovementExecutionMode movementExecutionMode,
            RandomGenerator startNodeRandom) {
        this(entity, service, profile, movementPerTick,
                false, movementExecutionMode, startNodeRandom);
    }

    EntityNavigationController(
            Entity entity, AsyncEntityPathfindingService service,
            NavigationProfile profile, double movementPerTick,
            MovementExecutionMode movementExecutionMode,
            ControllerRequestSubmitter controllerSubmitter,
            ControllerSchedulingOptions schedulingOptions) {
        this(entity, service, profile, movementPerTick, false,
                movementExecutionMode,
                NavigationRequestFactory.entityRandomOrNull(entity),
                controllerSubmitter, schedulingOptions);
    }

    /**
     * Submits straight to the service, bypassing the bounded admission layer
     * that {@link NavigationSystem}-owned controllers are given.
     */
    private EntityNavigationController(Entity entity,
                                       AsyncEntityPathfindingService service,
                                       NavigationProfile profile,
                                       double movementPerTick,
                                       boolean refreshBuiltinProfile,
                                       MovementExecutionMode movementExecutionMode,
                                       RandomGenerator startNodeRandom) {
        this(entity, service, profile, movementPerTick, refreshBuiltinProfile,
                movementExecutionMode, startNodeRandom,
                (key, request, priority) -> service.submitLatest(key, request),
                ControllerSchedulingOptions.direct(service.parallelism()));
    }

    private EntityNavigationController(Entity entity,
                                       AsyncEntityPathfindingService service,
                                       NavigationProfile profile,
                                       double movementPerTick,
                                       boolean refreshBuiltinProfile,
                                       MovementExecutionMode movementExecutionMode,
                                       RandomGenerator startNodeRandom,
                                       ControllerRequestSubmitter controllerSubmitter,
                                       ControllerSchedulingOptions schedulingOptions) {
        if (entity == null || service == null || profile == null
                || movementExecutionMode == null
                || startNodeRandom == null || controllerSubmitter == null
                || schedulingOptions == null
                || !Double.isFinite(movementPerTick) || movementPerTick <= 0) {
            throw new IllegalArgumentException("invalid controller arguments");
        }
        this.entity = entity;
        this.refreshBuiltinProfile = refreshBuiltinProfile;
        this.movementExecutionMode = movementExecutionMode;
        this.metrics = service.metrics();
        this.schedulingOptions = schedulingOptions;
        this.progress = new ProgressMonitor(movementPerTick);
        NavigationRequestFactory requests = new NavigationRequestFactory(
                entity, refreshBuiltinProfile, startNodeRandom);
        this.pathSubmitter = (key, instance, start, destination, box,
                              influences, modifiers) -> {
            NavigationProfile effective = snapshotDynamicTerrain(
                    modifiers.applyTo(currentBaseProfile()));
            NavigationRequest request = requests.build(
                    effective, instance, start, destination, box, influences,
                    replanner.searchRangeMultiplier(), geometry.isEntityInWater());
            return controllerSubmitter.submit(
                    key, request, routeExhausted() ? 100 : 10);
        };
        this.navigationProfile = profile;
        this.activeNavigationProfile = profile;
        this.movementPerTick = movementPerTick;
        this.wallClimberFallback =
                profile.mode() == NavigationMode.WALL_CLIMBER;
    }

    /**
     * Selects the baseline navigation profile for the entity type.
     */
    public static EntityNavigationController builtin(
            Entity entity, AsyncEntityPathfindingService service,
            double movementPerTick) {
        return builtin(entity, service, movementPerTick,
                NavigationRequestFactory.entityRandomOrNull(entity));
    }

    /**
     * Selects the baseline profile while sampling small-flyer starts from the
     * supplied entity AI random stream.
     */
    public static EntityNavigationController builtin(
            Entity entity, AsyncEntityPathfindingService service,
            double movementPerTick, RandomGenerator startNodeRandom) {
        return new EntityNavigationController(entity, service,
                BuiltinNavigationProfiles.forEntity(entity),
                movementPerTick, true,
                MovementExecutionMode.PHYSICS_VELOCITY, startNodeRandom);
    }

    static EntityNavigationController builtin(
            Entity entity, AsyncEntityPathfindingService service,
            double movementPerTick, ControllerRequestSubmitter submitter,
            ControllerSchedulingOptions options) {
        return builtin(entity, service, movementPerTick,
                NavigationRequestFactory.entityRandomOrNull(entity),
                submitter, options);
    }

    static EntityNavigationController builtin(
            Entity entity, AsyncEntityPathfindingService service,
            double movementPerTick, RandomGenerator random,
            ControllerRequestSubmitter submitter,
            ControllerSchedulingOptions options) {
        return new EntityNavigationController(entity, service,
                BuiltinNavigationProfiles.forEntity(entity), movementPerTick,
                true, MovementExecutionMode.PHYSICS_VELOCITY, random,
                submitter, options);
    }

    public void moveTo(Point newTarget) {
        moveTo(newTarget, List.of(), NavigationModifiers.NONE);
    }

    /**
     * Plans with request-scoped dynamic influences, such as entity fear,
     * temporary exclusion zones, or block preferences.
     */
    public void moveTo(Point newTarget, List<NavigationInfluence> influences) {
        moveTo(newTarget, influences, NavigationModifiers.NONE);
    }

    public void moveTo(Point newTarget, NavigationModifiers modifiers) {
        moveTo(newTarget, List.of(), modifiers);
    }

    /**
     * Plans using both dynamic world influences and immutable, request-scoped
     * changes to the entity's normal terrain costs or capabilities.
     */
    public void moveTo(Point newTarget, List<NavigationInfluence> influences,
                       NavigationModifiers modifiers) {
        Instance instance = entity.getInstance();
        if (instance == null || newTarget == null || influences == null
                || influences.stream().anyMatch(Objects::isNull)
                || modifiers == null) {
            transition(NavigationState.FAILED);
            return;
        }
        target = newTarget;
        travelRequired = !goalReached();
        targetUnreachable = false;
        unreachableRecorded = false;
        routeStop = PathStop.NOT_SEARCHED;
        activeInfluences = List.copyOf(influences);
        activeModifiers = modifiers;
        activeNavigationProfile = snapshotDynamicTerrain(
                modifiers.applyTo(currentBaseProfile()));
        if (geometry.movesThroughVolume()) {
            suppressGravity();
        }
        // Only moveTo() abandons the route the entity is on and stops the
        // motion that route was driving; follow() keeps both. A route that
        // can still be followed is kept until its replacement lands, so
        // retargeting every tick never leaves the entity standing still.
        if (routeExhausted()) {
            resetRoute();
            nodes = List.of();
            nodeIndex = 0;
            stopDrivenMotion();
            transition(NavigationState.COMPUTING);
        } else {
            replanner.reset(entity.getPosition().distance(target));
        }
        recomputePath(RouteReplanEvent.Reason.NEW_TARGET);
    }

    /**
     * Starts following a previously computed plan without submitting another
     * search. The plan must match this entity's dimensions and begin close to
     * its current position. Callers should obtain cached plans through a world
     * revision check before replaying them.
     */
    public void follow(NavigationPlan plan) {
        if (plan == null || !plan.usable()) {
            throw new IllegalArgumentException("usable plan required");
        }
        var entityBox = entity.getBoundingBox();
        var plannedBox = plan.boundingBox();
        if (Math.abs(entityBox.width() - plannedBox.width()) > 1.0e-6
                || Math.abs(entityBox.height() - plannedBox.height()) > 1.0e-6
                || Math.abs(entityBox.depth() - plannedBox.depth()) > 1.0e-6) {
            throw new IllegalArgumentException("plan bounding box does not match entity");
        }
        observeSearchLanding();
        PathPostProcessor.Result processed = postProcess(plan.nodes());
        // A plan that starts on the route already being followed needs no
        // start tolerance at all: its first node is one the entity walks
        // through, so the seam is exact rather than approximate.
        // The sun flag is not carried over, so a replayed plan is
        // never treated as sun-truncated however it is applied.
        boolean spliced = splicer.isArmed()
                && spliceInto(processed.nodes(), false);
        if (!spliced) {
            splicer.disarm();
            if (entity.getPosition().distance(plan.start()) > startTolerance()) {
                metrics.planTooStaleToFollow();
                throw new IllegalArgumentException(
                        "entity is too far from plan start");
            }
        }
        if (pending != null) pending.cancel(true);
        pending = null;
        target = plan.target();
        travelRequired = !goalReached();
        targetUnreachable = false;
        unreachableRecorded = false;
        // A replayed plan carries a status but no search reason, so the
        // chaining rules fall back to the terminal radius for it.
        routeStop = PathStop.NOT_SEARCHED;
        activeInfluences = List.of();
        activeModifiers = NavigationModifiers.NONE;
        activeNavigationProfile = plan.profile();
        if (geometry.movesThroughVolume()) suppressGravity();
        generation++;
        if (!spliced) {
            nodes = processed.nodes();
            nodeIndex = Math.min(1, nodes.size());
            // The reset follows the trim so a replayed plan is never treated
            // as sun-truncated.
            resetRoute();
        }
        transition(plan.status() == PathStatus.FOUND
                ? NavigationState.FOLLOWING : NavigationState.PARTIAL);
    }

    private void resetRoute() {
        progress.resetWaypoint();
        activePlatformJumpNode = -1;
        activePlatformJumpSpeed = 0;
        platformJumpAlignmentNode = -1;
        lastGroundImpulseNode = -1;
        pathTruncatedForSun = false;
        replanner.reset(entity.getPosition().distance(target));
        progress.resetProgress(entity.getPosition(), tick);
    }

    public void tick() {
        tick++;
        // A landed search is read before anything decides to replace it, so
        // completed work is never discarded on the tick it arrives.
        acceptPending();
        if (replanRequested) recomputePath(RouteReplanEvent.Reason.SHED_RETRY);
        // Closing runs before the route is followed and outside it, so a
        // manipulated block is still restored once the route ends.
        manipulator.closeBehind(entity.getInstance(),
                activeNavigationProfile.mobProfile(), entity.getPosition());
        if (target == null || state == NavigationState.COMPLETED
                || state == NavigationState.CANCELLED
                || state == NavigationState.STUCK
                || state == NavigationState.FAILED) {
            restoreGravity();
            stopDrivenMotion();
            return;
        }
        if (goalReached()) {
            restoreGravity();
            transition(NavigationState.COMPLETED);
            return;
        }
        if (state == NavigationState.FOLLOWING || state == NavigationState.PARTIAL) {
            followPath();
        } else if (wallClimberFallback && state != NavigationState.COMPUTING) {
            WALK.move(movementContext, target, true);
        }
        // A follower standing on a route the planner will not extend is not
        // stalled, so the watchdog must not relabel it as one.
        if (progress.stalled(entity.getPosition(), tick)
                && state != NavigationState.COMPUTING && !targetUnreachable) {
            transition(NavigationState.STUCK);
        }
    }

    /**
     * Rebuilds the current route, paced by search completion rather than by a
     * fixed number of ticks. A request arriving while a search is still in
     * flight is remembered and submitted once that search settles, so the
     * cadence follows however long searches actually take.
     */
    public void recomputePath() {
        recomputePath(RouteReplanEvent.Reason.REQUESTED);
    }

    private void recomputePath(RouteReplanEvent.Reason reason) {
        if (target == null) return;
        replanRequested = true;
        emit(RouteReplanEvent.class, () -> new RouteReplanEvent(this, reason));
        if (!searchSettledOrAbandoned()) return;
        Point destination = target;
        Instance instance = entity.getInstance();
        if (instance == null) {
            transition(NavigationState.FAILED);
            return;
        }
        lastSubmitTick = tick;
        replanRequested = false;
        activeNavigationProfile = snapshotDynamicTerrain(
                activeModifiers.applyTo(currentBaseProfile()));
        generation++;
        replanner.beginSegment(entity.getPosition().distance(destination));
        pending = pathSubmitter.submit(entity.getUuid(), instance,
                planningStart(destination), destination,
                entity.getBoundingBox(), activeInfluences, activeModifiers);
        // Keep following the last accepted route while the replacement is
        // computed. If no usable route exists, COMPUTING remains the only
        // meaningful state.
        if (routeExhausted()) {
            transition(NavigationState.COMPUTING);
            stopDrivenMotion();
        }
    }

    /**
     * Whether the search behind the current request may be replaced. Work in
     * flight is left alone until the tail this controller's service actually
     * observes says it will never land, so a chased mob keeps receiving
     * routes however slow searches become. The tail is read only once the
     * floor has already elapsed, so the ordinary path costs nothing.
     */
    private boolean searchSettledOrAbandoned() {
        if (tick <= lastSubmitTick || tick < shedBackoffUntilTick) return false;
        if (pending == null) return true;
        int waited = tick - lastSubmitTick;
        int minimumStallTicks = schedulingOptions.minimumSearchStallTicks();
        if (waited < minimumStallTicks) return false;
        long tail = metrics.snapshot().latency().p99Micros() / MICROS_PER_TICK;
        return waited >= Math.max(minimumStallTicks, tail * 4);
    }

    /**
     * The position a replacement route should be planned from: the node of
     * the current route the entity will have reached by the time the search
     * lands, rather than the position it occupies now. Arms the splice this
     * controller applies when the matching plan arrives, and falls back to the
     * entity's own position whenever no node can carry one.
     *
     * <p>Callers that submit their own searches for this controller, rather
     * than through {@link #recomputePath()}, must plan from this point and
     * hand the result to {@link #follow(NavigationPlan)}.</p>
     */
    public Point planningStart(Point destination) {
        splicer.disarm();
        planningStartTick = tick;
        Point position = entity.getPosition();
        // A changed block may have invalidated the very route a splice would
        // retain, so the world revision that requested this replan also
        // withdraws the entity's licence to keep walking the old one.
        if (spliceSuppressed) {
            spliceSuppressed = false;
            return position;
        }
        if (routeExhausted() || destination == null) return position;
        int latencyTicks = observedLatencyTicks();
        if (!RouteSplicer.worthSplicing(
                latencyTicks, movementPerTick, startTolerance())) {
            return position;
        }
        Point splice = splicer.arm(nodes, nodeIndex, position, destination,
                movementPerTick, latencyTicks, tick);
        return splice == null ? position : splice;
    }

    /**
     * Search latency this system actually observes, in whole ticks. Read only
     * when a route exists that could carry a splice, and only on the tick a
     * search is submitted, so following costs nothing.
     */
    private int observedLatencyTicks() {
        return splicer.expectedLatencyTicks((int) (metrics.snapshot()
                .latency().p90Micros() / MICROS_PER_TICK));
    }

    /** Records how many ticks the route just applied spent in flight. */
    private void observeSearchLanding() {
        if (planningStartTick < 0) return;
        splicer.observeLatency(tick - planningStartTick);
        planningStartTick = -1;
    }

    /**
     * Joins a landed route to the one still being followed at the node the
     * landed search was started from. The entity's position on the route is
     * unchanged, so the indices it holds are rebased rather than reset and
     * the waypoint it was walking toward stays the waypoint it walks toward.
     */
    private boolean spliceInto(List<PathNode> landed, boolean truncatedForSun) {
        RouteSplicer.Join join = splicer.join(nodes, nodeIndex, landed, tick);
        if (join == null) return false;
        nodes = join.nodes();
        nodeIndex -= join.offset();
        activePlatformJumpNode = rebase(activePlatformJumpNode, join.offset());
        platformJumpAlignmentNode =
                rebase(platformJumpAlignmentNode, join.offset());
        lastGroundImpulseNode = rebase(lastGroundImpulseNode, join.offset());
        pathTruncatedForSun = truncatedForSun;
        // Neither watchdog is reset: the entity is still walking toward the
        // same waypoint it was, so a splice must not extend the deadline of a
        // follower that has genuinely stopped making headway.
        return true;
    }

    private static int rebase(int nodeIndex, int offset) {
        return nodeIndex < 0 ? -1 : Math.max(-1, nodeIndex - offset);
    }

    /** How far from a plan's start this follower will still begin it. */
    private double startTolerance() {
        return Math.max(1.5, entity.getBoundingBox().width());
    }

    private boolean startsWithinTolerance(List<PathNode> route) {
        return !route.isEmpty() && entity.getPosition().distance(
                route.getFirst().asVec()) <= startTolerance();
    }

    /**
     * Withdraws the next replan's licence to keep walking the current route,
     * for callers whose own world revision says the retained stretch may no
     * longer be legal.
     */
    public void suppressNextSplice() {
        spliceSuppressed = true;
    }

    /** Splice bookkeeping, for tests and operators. */
    public RouteSplicer.Counts spliceCounts() {
        return splicer.counts();
    }

    private boolean routeExhausted() {
        return nodes.isEmpty() || nodeIndex >= nodes.size();
    }

    /**
     * Requests a recomputation when a changed block is close enough to affect
     * the unconsumed portion of the current path.
     */
    public void onBlockChanged(Point changedBlock) {
        if (changedBlock == null || nodes.isEmpty()
                || nodeIndex >= nodes.size()) return;
        PathNode end = nodes.getLast();
        Point position = entity.getPosition();
        Point middle = new Vec(
                (end.x() + position.x()) * 0.5,
                (end.y() + position.y()) * 0.5,
                (end.z() + position.z()) * 0.5);
        double remainingNodes = nodes.size() - nodeIndex;
        Point changedCenter = new Vec(
                changedBlock.blockX() + 0.5,
                changedBlock.blockY() + 0.5,
                changedBlock.blockZ() + 0.5);
        if (changedCenter.distance(middle) < remainingNodes) {
            // The changed block may lie on the very stretch a splice would
            // retain, so this replan starts from the entity itself.
            spliceSuppressed = true;
            recomputePath(RouteReplanEvent.Reason.BLOCK_CHANGE);
        }
    }

    private void acceptPending() {
        if (pending == null || !pending.isDone()) return;
        CompletableFuture<PathResult> completed = pending;
        pending = null;
        if (completed.isCancelled()) {
            transition(fallbackState(NavigationState.CANCELLED));
            return;
        }
        try {
            PathResult path = completed.join();
            // Backpressure, not a result: keep the route already being
            // followed and retry once the recompute limit allows.
            if (path.shed()) {
                replanRequested = true;
                shedBackoffUntilTick = tick + schedulingOptions.shedBackoffTicks();
                emit(PathShedEvent.class,
                        () -> new PathShedEvent(this, shedBackoffUntilTick));
                return;
            }
            adoptPath(path);
            emit(PathComputedEvent.class, () -> new PathComputedEvent(
                    this, path.status(), path.stop(), nodes));
            if (!wallClimberFallback && state == NavigationState.PARTIAL
                    && replanner.shouldExpandVerticalDetour(path, target)) {
                widenSearchRange();
            }
        } catch (CancellationException exception) {
            transition(fallbackState(NavigationState.CANCELLED));
        } catch (CompletionException exception) {
            transition(fallbackState(NavigationState.FAILED));
        }
    }

    private void adoptPath(PathResult path) {
        observeSearchLanding();
        routeStop = path.stop();
        targetUnreachable = false;
        PathPostProcessor.Result processed = postProcess(path.nodes());
        boolean routed = path.status() == PathStatus.FOUND
                || path.status() == PathStatus.PARTIAL
                || path.status() == PathStatus.TIMED_OUT;
        boolean spliceRequested = splicer.isArmed();
        if (routed && spliceInto(processed.nodes(),
                processed.truncatedForSun())) {
            transition(path.status() == PathStatus.FOUND
                    ? NavigationState.FOLLOWING : NavigationState.PARTIAL);
            return;
        }
        splicer.disarm();
        // A refused splice leaves a route that begins where the entity was
        // going to be rather than where it is, so adopting it would install
        // the very mismatch splicing exists to remove. The route already
        // being followed is kept instead and a replacement is planned from
        // the entity itself. An entity with no route left still takes it: a
        // route it must walk to beats no route at all.
        if (spliceRequested && routed && !routeExhausted()
                && !startsWithinTolerance(processed.nodes())) {
            replanRequested = true;
            return;
        }
        nodes = processed.nodes();
        pathTruncatedForSun = processed.truncatedForSun();
        nodeIndex = Math.min(1, nodes.size());
        progress.resetWaypoint();
        // Route construction can span more than one progress window. The
        // first movement command is applied by Minestom after this
        // controller tick, so retaining the pre-search timestamp could
        // declare the entity stuck before that command moves it.
        progress.resetProgress(entity.getPosition(), tick);
        transition(switch (path.status()) {
            case FOUND -> NavigationState.FOLLOWING;
            case PARTIAL, TIMED_OUT -> NavigationState.PARTIAL;
            case CANCELLED -> NavigationState.CANCELLED;
            default -> fallbackState(NavigationState.FAILED);
        });
    }

    private PathPostProcessor.Result postProcess(List<PathNode> path) {
        return PathPostProcessor.process(entity.getInstance(),
                activeNavigationProfile, entity.getPosition(), path);
    }

    /**
     * Doubles the search budget and restarts from an empty route.
     */
    private void widenSearchRange() {
        replanner.widen();
        nodes = List.of();
        nodeIndex = 0;
        recomputePath(RouteReplanEvent.Reason.PARTIAL_CONTINUATION);
    }

    private NavigationState fallbackState(NavigationState terminal) {
        return wallClimberFallback ? NavigationState.PARTIAL : terminal;
    }

    /**
     * Applies a state change and reports the outcome of one navigation exactly
     * once, when an active navigation reaches a terminal state. Re-targeting an
     * unfinished route is not an outcome.
     */
    private void transition(NavigationState next) {
        if (state == next) return;
        boolean active = switch (state) {
            case IDLE, COMPUTING, FOLLOWING, PARTIAL -> true;
            case COMPLETED, STUCK, FAILED, CANCELLED -> false;
        };
        state = next;
        if (active) metrics.controllerOutcome(next, travelRequired);
        // After the state is published, so a listener reading state() sees the
        // state it was told about rather than the one being left.
        switch (next) {
            case COMPLETED -> emit(NavigationCompletedEvent.class,
                    () -> new NavigationCompletedEvent(this));
            case STUCK -> emit(NavigationStuckEvent.class,
                    () -> new NavigationStuckEvent(this));
            case FAILED -> emit(NavigationFailedEvent.class,
                    () -> new NavigationFailedEvent(this));
            case CANCELLED -> emit(NavigationCancelledEvent.class,
                    () -> new NavigationCancelledEvent(this));
            case IDLE, COMPUTING, FOLLOWING, PARTIAL -> {
            }
        }
    }

    private void followPath() {
        if (nodeIndex >= nodes.size()) {
            finishConsumedRoute();
            return;
        }
        PathNode node = nodes.get(nodeIndex);
        Point waypoint = node.asVec();
        manipulator.openAt(entity.getInstance(),
                activeNavigationProfile.mobProfile(), waypoint,
                entity.getBoundingBox());
        Point position = geometry.waypointTrackingPosition();
        boolean climbableWaypoint = geometry.isClimbableWaypoint(node);
        PathNode next = nodeIndex + 1 < nodes.size()
                ? nodes.get(nodeIndex + 1) : null;
        if (geometry.reachedWaypoint(node, waypoint, position, climbableWaypoint)
                || geometry.canSkipAheadOf(node, next, position)) {
            consumeWaypoint();
            return;
        }
        progress.armWaypointTimeout(position, waypoint);
        driveWaypointMovement(node, waypoint, climbableWaypoint);
        if (progress.waypointTimedOut()) transition(NavigationState.STUCK);
    }

    private void finishConsumedRoute() {
        if (pathTruncatedForSun) {
            transition(NavigationState.COMPLETED);
            return;
        }
        if (state == NavigationState.PARTIAL) {
            if (wallClimberFallback) {
                WALK.move(movementContext, target, true);
                return;
            }
            if (continueLongPartialRoute()) return;
        }
        restoreGravity();
        stopDrivenMotion();
        if (state != NavigationState.PARTIAL) {
            transition(NavigationState.COMPLETED);
        }
    }

    private void consumeWaypoint() {
        nodeIndex++;
        activePlatformJumpNode = -1;
        activePlatformJumpSpeed = 0;
        platformJumpAlignmentNode = -1;
        progress.resetWaypoint();
    }

    private void driveWaypointMovement(PathNode node, Point waypoint,
                                       boolean climbableWaypoint) {
        if (node.movement() == PathNode.Movement.JUMP) {
            restoreGravity();
            PLATFORM_JUMP.move(movementContext, waypoint);
        } else if (node.movement() == PathNode.Movement.CLIMB) {
            suppressGravity();
            CLIMB.move(movementContext, waypoint);
        } else if (geometry.isSpatialNode(node)) {
            suppressGravity();
            VOLUME.move(movementContext, waypoint);
        } else {
            restoreGravity();
            if (climbableWaypoint) {
                CLIMB.moveIntoClimbable(movementContext, waypoint);
            } else {
                WALK.move(movementContext, waypoint, wallClimberFallback);
            }
        }
    }

    private boolean continueLongPartialRoute() {
        return switch (replanner.continueLongPartialRoute(
                entity.getPosition(), target, nodes, routeStop)) {
            case WIDEN -> {
                widenSearchRange();
                yield true;
            }
            case REPLAN -> {
                recomputePath(RouteReplanEvent.Reason.PARTIAL_CONTINUATION);
                yield true;
            }
            case STOP -> {
                recordTerminalPartial();
                yield false;
            }
        };
    }

    /**
     * Records, once per navigation, that no further search will bring this
     * follower closer to its target. Taken where that is decided rather than
     * left to the stall watchdog, which two hundred ticks later could only
     * call it a stall. The counter is held to one per navigation while the
     * flag tracks the live situation, so a route landing afterwards clears
     * what a caller reads without double-counting the navigation.
     */
    private void recordTerminalPartial() {
        targetUnreachable = true;
        if (unreachableRecorded) return;
        unreachableRecorded = true;
        emit(TargetUnreachableEvent.class,
                () -> new TargetUnreachableEvent(this));
        metrics.controllerTargetUnreachable();
    }

    private void stopDrivenMotion() {
        if (movementExecutionMode != MovementExecutionMode.PHYSICS_VELOCITY) {
            return;
        }
        Vec velocity = entity.getVelocity();
        entity.setVelocity(geometry.movesThroughVolume()
                ? Vec.ZERO : new Vec(0, velocity.y(), 0));
    }

    private void suppressGravity() {
        if (gravityOverridden) return;
        previousNoGravity = entity.hasNoGravity();
        entity.setNoGravity(true);
        gravityOverridden = true;
    }

    private void restoreGravity() {
        if (!gravityOverridden) return;
        entity.setNoGravity(previousNoGravity);
        gravityOverridden = false;
    }

    private boolean goalReached() {
        double tolerance = geometry.alwaysMovesThroughVolume() ? 0.7 : 0.5;
        Point effectiveTarget = target;
        if (target != null && geometry.alwaysMovesThroughVolume()) {
            effectiveTarget = new Vec(
                    Math.floor(target.x()) + 0.5,
                    Math.floor(target.y()),
                    Math.floor(target.z()) + 0.5);
        }
        return effectiveTarget != null && entity.getPosition().distance(effectiveTarget)
                <= Math.max(tolerance, entity.getBoundingBox().width() / 2);
    }

    private NavigationProfile currentBaseProfile() {
        NavigationProfile base = refreshBuiltinProfile
                ? BuiltinNavigationProfiles.forEntity(entity)
                : navigationProfile;
        Entity vehicle = entity.getVehicle();
        return BuiltinNavigationProfiles.withVehicleTerrainProfile(
                base, vehicle == null ? null : vehicle.getEntityType());
    }

    private NavigationProfile snapshotDynamicTerrain(NavigationProfile profile) {
        return BuiltinNavigationProfiles.withLiveTerrainState(
                profile, entity.getEntityType(),
                entity.isOnFire(), geometry.isEntityInWater());
    }

    void eventSink(NavigationEventSink sink) {
        events = sink == null ? NavigationEventSink.NONE : sink;
    }

    private void emit(Class<? extends NavigationEvent> type,
                      java.util.function.Supplier<NavigationEvent> event) {
        if (events.listening(type)) events.emit(event.get());
    }

    /** The entity this controller drives. */
    public Entity entity() {
        return entity;
    }

    public NavigationState state() {
        return state;
    }

    /**
     * Whether the planner has stopped offering segments toward the current
     * target. Read it beside {@link #state()} to tell the two ways a follower
     * goes quiet apart: {@code PARTIAL} with this set means the search cannot
     * reach the goal from where the entity stands and the caller should pick
     * another one, while {@link NavigationState#STUCK} means the entity is
     * physically wedged on a route it was given. It is only proof of
     * unreachability when the search emptied its frontier; otherwise it is the
     * planner declining to spend another bounded segment on this target.
     *
     * <p>Cleared by {@link #moveTo(Point)}, by {@link #follow(NavigationPlan)},
     * and by any route that lands afterwards.</p>
     */
    public boolean targetUnreachable() {
        return targetUnreachable;
    }

    /** Why the search behind the current route stopped. */
    public PathStop routeStop() {
        return routeStop;
    }

    /**
     * Cumulative telemetry of the service this controller plans with. Outcomes
     * are counted here rather than sampled from a controller registry, so no
     * reference to any controller is retained anywhere.
     */
    public NavigationMetrics metrics() {
        return metrics;
    }

    public int generation() {
        return generation;
    }

    public int nodeIndex() {
        return nodeIndex;
    }

    public List<PathNode> nodes() {
        return nodes;
    }

    @Override
    public void close() {
        if (pending != null) pending.cancel(true);
        restoreGravity();
        // A controller closed before it ever navigated has no outcome.
        if (state == NavigationState.IDLE) state = NavigationState.CANCELLED;
        else transition(NavigationState.CANCELLED);
    }

    private final class ControllerMovementContext implements MovementContext {
        @Override
        public Entity entity() {
            return entity;
        }

        @Override
        public double movementPerTick() {
            return movementPerTick;
        }

        @Override
        public MovementExecutionMode executionMode() {
            return movementExecutionMode;
        }

        @Override
        public NavigationProfile profile() {
            return activeNavigationProfile;
        }

        @Override
        public PathNode currentNode() {
            return nodeIndex < nodes.size() ? nodes.get(nodeIndex) : null;
        }

        @Override
        public PathNode previousNode() {
            return nodeIndex > 0 && nodeIndex <= nodes.size()
                    ? nodes.get(nodeIndex - 1) : null;
        }

        @Override
        public boolean wallClimberFallback() {
            return wallClimberFallback;
        }

        @Override
        public boolean claimGroundImpulse() {
            if (lastGroundImpulseNode == nodeIndex) return false;
            lastGroundImpulseNode = nodeIndex;
            return true;
        }

        @Override
        public boolean platformJumpStarted() {
            return activePlatformJumpNode == nodeIndex;
        }

        @Override
        public int countPlatformJumpAlignment() {
            if (platformJumpAlignmentNode != nodeIndex) {
                platformJumpAlignmentNode = nodeIndex;
                platformJumpAlignmentTicks = 0;
            }
            return ++platformJumpAlignmentTicks;
        }

        @Override
        public void startPlatformJump(double horizontalSpeed) {
            activePlatformJumpSpeed = horizontalSpeed;
            activePlatformJumpNode = nodeIndex;
        }

        @Override
        public double platformJumpSpeed() {
            return activePlatformJumpSpeed;
        }

        @Override
        public void requestRecompute() {
            recomputePath(RouteReplanEvent.Reason.REQUESTED);
        }
    }
}
