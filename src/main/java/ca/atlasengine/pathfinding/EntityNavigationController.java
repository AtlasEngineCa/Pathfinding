package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.event.NavigationCancelledEvent;
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
import ca.atlasengine.pathfinding.internal.RouteLength;
import ca.atlasengine.pathfinding.internal.SearchLatency;
import ca.atlasengine.pathfinding.internal.movement.BlockManipulator;
import ca.atlasengine.pathfinding.internal.movement.ClimbMovementExecutor;
import ca.atlasengine.pathfinding.internal.movement.MovementContext;
import ca.atlasengine.pathfinding.internal.movement.MovementExecutor;
import ca.atlasengine.pathfinding.internal.movement.NavigationRequestFactory;
import ca.atlasengine.pathfinding.internal.movement.PartialRouteReplanner;
import ca.atlasengine.pathfinding.internal.movement.PathPostProcessor;
import ca.atlasengine.pathfinding.internal.movement.PlatformJumpMovementExecutor;
import ca.atlasengine.pathfinding.internal.movement.ProgressMonitor;
import ca.atlasengine.pathfinding.internal.movement.RouteGeometry;
import ca.atlasengine.pathfinding.internal.movement.RouteSplicer;
import ca.atlasengine.pathfinding.internal.movement.VolumeMovementExecutor;
import ca.atlasengine.pathfinding.internal.movement.WalkMovementExecutor;
import ca.atlasengine.pathfinding.metrics.NavigationMetrics;
import ca.atlasengine.pathfinding.metrics.SpliceCounts;
import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.profile.NavigationModifiers;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.result.PathStatus;
import ca.atlasengine.pathfinding.result.PathStop;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;
import net.minestom.server.instance.Instance;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.CancellationException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionException;
import java.util.function.Supplier;
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
    /** Submission priority of a replan that leaves the entity with no route. */
    private static final int ROUTELESS_PRIORITY = 100;
    private static final int REPLAN_PRIORITY = 10;
    private static final WalkMovementExecutor WALK = new WalkMovementExecutor();
    private static final ClimbMovementExecutor CLIMB = new ClimbMovementExecutor();
    private static final MovementExecutor VOLUME = new VolumeMovementExecutor();
    private static final MovementExecutor PLATFORM_JUMP =
            new PlatformJumpMovementExecutor();

    private final Entity entity;
    private final NavigationRequestFactory requests;
    private final ControllerRequestSubmitter submitter;
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
    private boolean partialContinuationPrefetched;
    private boolean waypointRecoveryAttempted;
    private int activePlatformJumpNode = -1;
    private double activePlatformJumpSpeed;
    private int platformJumpAlignmentNode = -1;
    private int platformJumpAlignmentTicks;
    private int lastGroundImpulseNode = -1;
    /** Set from any thread by {@link #blockChanged}, consumed by tick(). */
    private volatile boolean blockChangePending;

    /**
     * Controllers are created by {@link NavigationSystem#controller(Entity)}
     * and {@link NavigationSystem#controllerBuilder(Entity)}, which wire them
     * to the system's admission layer and event node.
     *
     * @param refreshBuiltinProfile re-derive the built-in profile from the
     *                              entity's live metadata for every search
     *                              rather than keep {@code profile}
     */
    EntityNavigationController(Entity entity,
                               NavigationMetrics metrics,
                               NavigationProfile profile,
                               double movementPerTick,
                               boolean refreshBuiltinProfile,
                               MovementExecutionMode movementExecutionMode,
                               RandomGenerator startNodeRandom,
                               ControllerRequestSubmitter submitter,
                               ControllerSchedulingOptions schedulingOptions) {
        if (entity == null || metrics == null || profile == null
                || movementExecutionMode == null
                || startNodeRandom == null || submitter == null
                || schedulingOptions == null
                || !Double.isFinite(movementPerTick) || movementPerTick <= 0) {
            throw new IllegalArgumentException("invalid controller arguments");
        }
        this.entity = entity;
        this.refreshBuiltinProfile = refreshBuiltinProfile;
        this.movementExecutionMode = movementExecutionMode;
        this.metrics = metrics;
        this.schedulingOptions = schedulingOptions;
        this.progress = new ProgressMonitor(movementPerTick);
        this.requests = new NavigationRequestFactory(
                entity, refreshBuiltinProfile, startNodeRandom);
        this.submitter = submitter;
        this.navigationProfile = profile;
        this.activeNavigationProfile = profile;
        this.movementPerTick = movementPerTick;
        this.wallClimberFallback =
                profile.mode() == NavigationMode.WALL_CLIMBER;
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
        beginNavigation(newTarget, List.copyOf(influences), modifiers,
                snapshotDynamicTerrain(modifiers.applyTo(currentBaseProfile())));
        // Only moveTo() abandons the route the entity is on and stops the
        // motion that route was driving; follow() keeps both. A route that
        // can still be followed is kept until its replacement lands, so
        // retargeting every tick never leaves the entity standing still.
        if (routeExhausted()) {
            resetRoute();
            dropRoute();
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
        if (!sameDimensions(entity.getBoundingBox(), plan.boundingBox())) {
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
        cancelPending();
        beginNavigation(plan.target(), List.of(), NavigationModifiers.NONE,
                plan.profile());
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

    /**
     * Resets everything a new navigation inherits from the last one. The
     * search reason starts unknown: a replayed plan carries a status but no
     * reason, so the chaining rules fall back to the terminal radius for it.
     */
    private void beginNavigation(Point newTarget,
                                 List<NavigationInfluence> influences,
                                 NavigationModifiers modifiers,
                                 NavigationProfile profile) {
        target = newTarget;
        // Judged before the profile changes, against the one being left.
        travelRequired = !goalReached();
        targetUnreachable = false;
        unreachableRecorded = false;
        waypointRecoveryAttempted = false;
        routeStop = PathStop.NOT_SEARCHED;
        activeInfluences = influences;
        activeModifiers = modifiers;
        activeNavigationProfile = profile;
        if (geometry.movesThroughVolume()) suppressGravity();
    }

    private static boolean sameDimensions(BoundingBox first,
                                          BoundingBox second) {
        return Math.abs(first.width() - second.width()) <= 1.0e-6
                && Math.abs(first.height() - second.height()) <= 1.0e-6
                && Math.abs(first.depth() - second.depth()) <= 1.0e-6;
    }

    private void cancelPending() {
        if (pending != null) pending.cancel(true);
        pending = null;
    }

    private void dropRoute() {
        nodes = List.of();
        nodeIndex = 0;
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
        // Chunk unloading removes non-player entities before owners necessarily
        // discard their controllers. Never let a late search result or movement
        // command touch Minestom's detached collision state.
        if (entity.isRemoved() || entity.getInstance() == null) {
            cancelPending();
            cancelNavigation();
            return;
        }
        // A landed search is read before anything decides to replace it, so
        // completed work is never discarded on the tick it arrives.
        acceptPending();
        if (blockChangePending) {
            blockChangePending = false;
            if (!routeExhausted()) replanForBlockChange();
        }
        if (replanRequested) recomputePath(RouteReplanEvent.Reason.SHED_RETRY);
        // Closing runs before the route is followed and outside it, so a
        // manipulated block is still restored once the route ends.
        manipulator.closeBehind(entity.getInstance(),
                activeNavigationProfile.mobProfile(), entity.getPosition());
        if (target == null || state.terminal()) {
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
            prefetchPartialContinuation();
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
        NavigationRequest request = requests.build(activeNavigationProfile,
                instance, planningStart(destination), destination,
                entity.getBoundingBox(), activeInfluences,
                replanner.searchRangeMultiplier(), geometry.isEntityInWater());
        pending = submitter.submit(entity.getUuid(), request,
                routeExhausted() ? ROUTELESS_PRIORITY : REPLAN_PRIORITY);
        // Keep following the last accepted route while the replacement is
        // computed. If no usable route exists, COMPUTING remains the only
        // meaningful state.
        if (routeExhausted()) {
            transition(NavigationState.COMPUTING);
            stopDrivenMotion();
        }
    }

    /** Whether the search behind the current request may be replaced. */
    private boolean searchSettledOrAbandoned() {
        if (tick <= lastSubmitTick || tick < shedBackoffUntilTick) return false;
        return pending == null || SearchLatency.abandoned(
                tick - lastSubmitTick,
                schedulingOptions.minimumSearchStallTicks(), metrics);
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
        return splicer.expectedLatencyTicks((int) SearchLatency.ticks(
                metrics.snapshot().latency().p90Micros()));
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
        int joinedIndex = nodeIndex - join.offset();
        Point trackingPosition = geometry.waypointTrackingPosition();
        // The follower can physically shortcut beyond an armed seam without
        // consuming every graph waypoint leading to it. In that case index
        // rebasing is arithmetically valid but the retained current waypoint
        // lies behind the entity, sometimes across the wall it just passed.
        // Refuse that stale prefix and let adoption request a route from the
        // entity's live position instead of steering backwards into it.
        if (joinedIndex < 0 || joinedIndex >= join.nodes().size()
                || trackingPosition.distance(
                join.nodes().get(joinedIndex).asVec()) > startTolerance()
                || !geometry.canSweepTo(trackingPosition,
                join.nodes().get(joinedIndex).asVec())) {
            return false;
        }
        nodes = join.nodes();
        nodeIndex = joinedIndex;
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
    public SpliceCounts spliceCounts() {
        return splicer.counts();
    }

    private boolean routeExhausted() {
        return nodes.isEmpty() || nodeIndex >= nodes.size();
    }

    /**
     * Requests a recomputation when a changed block is close enough to affect
     * the unconsumed portion of the current path. Call it from the thread
     * that ticks this controller.
     */
    public void onBlockChanged(Point changedBlock) {
        if (changedBlock != null && routeMayCross(changedBlock)) {
            replanForBlockChange();
        }
    }

    /**
     * The thread-safe form {@link NavigationSystem} uses: the edit may come
     * from any thread and any instance, so it is only judged here and acted
     * on by the next {@link #tick()}, which is also where its event fires.
     */
    void blockChanged(Instance instance, Point changedBlock) {
        if (entity.getInstance() == instance && routeMayCross(changedBlock)) {
            blockChangePending = true;
        }
    }

    /**
     * Whether a block lies near enough to the stretch still to be walked to
     * matter. Reads only the immutable route and the entity position, so a
     * stale answer from another thread errs toward a harmless extra replan.
     */
    private boolean routeMayCross(Point changedBlock) {
        List<PathNode> route = nodes;
        int remaining = route.size() - nodeIndex;
        if (remaining <= 0) return false;
        PathNode end = route.getLast();
        Point position = entity.getPosition();
        Point middle = new Vec(
                (end.x() + position.x()) * 0.5,
                (end.y() + position.y()) * 0.5,
                (end.z() + position.z()) * 0.5);
        Point changedCenter = new Vec(
                changedBlock.blockX() + 0.5,
                changedBlock.blockY() + 0.5,
                changedBlock.blockZ() + 0.5);
        return changedCenter.distance(middle) < remaining;
    }

    private void replanForBlockChange() {
        // The changed block may lie on the very stretch a splice would
        // retain, so this replan starts from the entity itself.
        spliceSuppressed = true;
        recomputePath(RouteReplanEvent.Reason.BLOCK_CHANGE);
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
        partialContinuationPrefetched = false;
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
        dropRoute();
        recomputePath(RouteReplanEvent.Reason.PARTIAL_CONTINUATION);
    }

    /**
     * Starts the next bounded segment while this one still has enough route to
     * cover the expected search wait. The accepted tail remains active and the
     * landed result uses the normal splice path, so computation never creates
     * an intentional stop at a partial endpoint.
     */
    private void prefetchPartialContinuation() {
        if (state != NavigationState.PARTIAL
                || wallClimberFallback || partialContinuationPrefetched
                || pending != null
                || replanRequested || target == null || routeExhausted()
                || pathTruncatedForSun) {
            return;
        }
        double safetyTail = Math.max(8,
                (observedLatencyTicks() + 2) * movementPerTick);
        if (remainingRouteDistance() > safetyTail) return;

        partialContinuationPrefetched = true;
        switch (replanner.continueLongPartialRoute(
                nodes.getLast().asVec(), target, nodes, routeStop)) {
            case WIDEN -> {
                replanner.widen();
                recomputePath(RouteReplanEvent.Reason.PARTIAL_CONTINUATION);
            }
            case REPLAN ->
                    recomputePath(RouteReplanEvent.Reason.PARTIAL_CONTINUATION);
            case STOP -> {
                // The endpoint remains usable. Terminal state is published
                // only after the follower actually consumes it.
            }
        }
    }

    private double remainingRouteDistance() {
        return RouteLength.from(entity.getPosition(), nodes, nodeIndex);
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
        boolean active = !state.terminal();
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
        if (geometry.reachedWaypoint(
                node, next, waypoint, position, climbableWaypoint)
                || geometry.canSkipAheadOf(node, next, position)) {
            consumeWaypoint();
            return;
        }
        progress.armWaypointTimeout(position, waypoint);
        driveWaypointMovement(node, waypoint, climbableWaypoint);
        if (progress.waypointTimedOut()) {
            if (!waypointRecoveryAttempted) {
                waypointRecoveryAttempted = true;
                splicer.disarm();
                spliceSuppressed = true;
                dropRoute();
                resetRoute();
                stopDrivenMotion();
                transition(NavigationState.COMPUTING);
                recomputePath(RouteReplanEvent.Reason.REQUESTED);
            } else {
                transition(NavigationState.STUCK);
            }
        }
    }

    private void finishConsumedRoute() {
        if (pathTruncatedForSun) {
            transition(NavigationState.COMPLETED);
            return;
        }
        if (state == NavigationState.PARTIAL) {
            if (pending != null || replanRequested) {
                transition(NavigationState.COMPUTING);
                stopDrivenMotion();
                return;
            }
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
        if (target == null) return false;
        // A volume navigator is steered to its target cell's bottom centre.
        boolean volume = geometry.alwaysMovesThroughVolume();
        Point goal = volume
                ? new Vec(Math.floor(target.x()) + 0.5, Math.floor(target.y()),
                        Math.floor(target.z()) + 0.5)
                : target;
        double tolerance = Math.max(volume ? 0.7 : 0.5,
                entity.getBoundingBox().width() / 2);
        return entity.getPosition().distance(goal) <= tolerance;
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
                      Supplier<NavigationEvent> event) {
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
     * unreachability when {@link #routeStop()} reads
     * {@link PathStop#FRONTIER_EXHAUSTED}; otherwise it is the planner
     * declining to spend another bounded segment on this target. In
     * particular {@link PathStop#TERRAIN_UNLOADED} means the route ran into
     * chunks that are not loaded: call {@link #moveTo(Point)} again once they
     * are.
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

    /**
     * Abandons any search in flight and restores gravity. A navigation still
     * in progress ends {@link NavigationState#CANCELLED}; one that already
     * completed, stalled, or failed keeps that outcome.
     */
    @Override
    public void close() {
        cancelPending();
        restoreGravity();
        cancelNavigation();
    }

    /**
     * Ends the navigation in progress as cancelled. A controller that never
     * navigated has no outcome to report, and one whose navigation already
     * finished keeps the outcome it reported.
     */
    private void cancelNavigation() {
        if (state == NavigationState.IDLE) {
            state = NavigationState.CANCELLED;
        } else if (!state.terminal()) {
            transition(NavigationState.CANCELLED);
        }
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
