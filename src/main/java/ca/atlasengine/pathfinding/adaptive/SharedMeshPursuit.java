package ca.atlasengine.pathfinding.adaptive;

import net.minestom.server.instance.block.Block;
import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.internal.adaptive.TargetCell;
import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.EntityNavigationController;
import ca.atlasengine.pathfinding.search.EntityTraversalState;
import ca.atlasengine.pathfinding.profile.NavigationModifiers;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.NavigationState;
import ca.atlasengine.pathfinding.NavigationSystem;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.result.PathStatus;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.internal.movement.RouteSplicer;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import net.minestom.server.coordinate.Point;
import net.minestom.server.entity.attribute.Attribute;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.LivingEntity;
import net.minestom.server.instance.Instance;

import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.CancellationException;
import java.util.concurrent.CompletionException;

/**
 * Tick-thread owner that connects an entity pair to adaptive planning and a
 * normal path follower. Target movement replaces only this actor's membership;
 * the regional mesh and other target fields remain reusable.
 *
 * <p>This type owns its follower and drives it: call {@link #tick(long, long)}
 * once per game tick and nothing else. That is the opposite contract to a plain
 * {@link EntityNavigationController}, which the caller creates and ticks
 * itself, so the follower underneath a pursuit is never handed
 * out; {@link #state()}, {@link #nodes()}, {@link #generation()}, and {@link
 * #spliceCounts()} report everything it can be asked. A pursuit driven from
 * more than one place is safe as well: a repeated tick id advances nothing a
 * second time, so a mob can never be walked twice in one tick.</p>
 *
 * <p>A pursued target leaves its block cell far more often than a search
 * completes, so a moved target never supersedes a search already in flight:
 * the route is applied first, and the replacement is submitted afterwards.
 * Replanning is therefore paced by search completion rather than by target
 * movement, which is what keeps a chased mob supplied with routes at any
 * search latency. Only a changed world revision, whose retained topology the
 * search may already have read, abandons work in flight.</p>
 */
public final class SharedMeshPursuit implements AutoCloseable {
    private static final long MICROS_PER_TICK = 50_000;

    private final NavigationSystem system;
    private final Entity actor;
    private final Entity target;
    private final EntityNavigationController controller;
    private final List<NavigationInfluence> influences;
    private final NavigationModifiers modifiers;
    private SharedMeshHandle planning;
    private TargetCell requestedTargetCell;
    private long worldRevision;
    private long currentTick;
    private long advancedTick = Long.MIN_VALUE;
    private long submittedTick;
    private boolean consumed;
    private boolean following;
    private boolean replanRequested;
    private long earliestReplanTick;
    private boolean closed;

    public SharedMeshPursuit(
            NavigationSystem system, Entity actor, Entity target,
            long worldRevision, long currentTick,
            List<NavigationInfluence> influences,
            NavigationModifiers modifiers) {
        if (system == null || actor == null || target == null
                || influences == null || modifiers == null
                || influences.stream().anyMatch(Objects::isNull)
                || currentTick < 0) {
            throw new IllegalArgumentException("invalid adaptive entity navigation");
        }
        this.system = system;
        this.actor = actor;
        this.target = target;
        this.worldRevision = worldRevision;
        this.currentTick = currentTick;
        this.influences = List.copyOf(influences);
        this.modifiers = modifiers;
        controller = system.controller(actor);
        submit();
    }

    /**
     * Advances planning and following from the instance tick thread. The world
     * revision must change whenever retained collision topology becomes stale.
     *
     * <p>One tick id advances this pursuit once. A repeated id returns without
     * touching planning or the follower, and without recording its revision, so
     * a world edit announced by the swallowed call is still seen by the next
     * genuine tick.</p>
     */
    public void tick(long worldRevision, long currentTick) {
        if (closed) return;
        if (currentTick < this.currentTick) {
            throw new IllegalArgumentException("ticks must be monotonic");
        }
        if (currentTick == advancedTick) return;
        advancedTick = currentTick;
        boolean revisionChanged = worldRevision != this.worldRevision;
        this.currentTick = currentTick;
        this.worldRevision = worldRevision;
        if (!entitiesRemainCompatible()) {
            if (target.isRemoved() || isDead(target)) {
                system.sharedMesh().forgetTarget(target.getUuid());
            }
            close();
            return;
        }
        // A landed search is consumed before anything decides to replace it,
        // so completed work is never discarded on the tick it arrives.
        if (!consumed && planning.plan().isDone()) applyCompletedPlan();
        if (revisionChanged) {
            system.sharedMesh().invalidateWorld(
                    actor.getInstance(), worldRevision, currentTick);
            // The retained collision topology this revision invalidates is the
            // same topology the current route was planned against, so this
            // replan starts from the actor rather than from a node on it.
            controller.suppressNextSplice();
            submit();
        } else {
            if (!TargetCell.of(target.getPosition())
                    .equals(requestedTargetCell)) replanRequested = true;
            if (replanRequested && currentTick >= earliestReplanTick
                    && searchSettledOrAbandoned()) submit();
        }
        if (closed) return;
        // The route already being followed outlives the search that replaces
        // it. A slightly stale route toward where the target was beats no
        // route at all, and the follower still refuses one it cannot start.
        if (following) controller.tick();
    }

    public Entity actor() {
        return actor;
    }

    public Entity target() {
        return target;
    }

    /** State of the follower this pursuit owns and ticks. */
    public NavigationState state() {
        return controller.state();
    }

    /** The route the follower is walking, or an empty list while it has none. */
    public List<PathNode> nodes() {
        return controller.nodes();
    }

    /** Whether no search reaches this target, as opposed to a wedged mob. */
    public boolean targetUnreachable() {
        return controller.targetUnreachable();
    }

    /** Counts the routes the follower has accepted, for liveness assertions. */
    public int generation() {
        return controller.generation();
    }

    /** Splice bookkeeping of the follower, for tests and operators. */
    public RouteSplicer.Counts spliceCounts() {
        return controller.spliceCounts();
    }

    public SharedMeshSource source() {
        return planning.source();
    }

    public boolean closed() {
        return closed;
    }

    @Override
    public void close() {
        if (closed) return;
        closed = true;
        if (planning != null) planning.close();
        controller.close();
    }

    private void submit() {
        if (!entitiesRemainCompatible()) {
            close();
            return;
        }
        if (planning != null) planning.close();
        Instance instance = actor.getInstance();
        NavigationProfile base = BuiltinNavigationProfiles.forEntity(actor);
        NavigationProfile profile = modifiers.applyTo(base);
        double range = actor instanceof LivingEntity living
                ? living.getAttribute(Attribute.FOLLOW_RANGE).getValue() : 16;
        range = Math.max(range,
                BuiltinNavigationProfiles.requiredPathLength(actor));
        // Plan from where the actor will stand when this search lands, not
        // from where it stands now. The follower arms the matching splice and
        // hands back the actor's own position whenever it cannot carry one.
        Point start = controller.planningStart(target.getPosition());
        boolean inWater = BlockTraversalData.hasWaterFluid(
                instance.getBlock(start));
        Set<Block> standable = profile.mobProfile().standsOnLava()
                ? Set.of(Block.LAVA) : Set.of();
        EntityTraversalState state = new EntityTraversalState(
                actor.isOnGround(), inWater, standable,
                instance.getCachedDimensionType().minY(),
                instance.getCachedDimensionType().maxY(), List.of());
        NavigationRequest request = NavigationRequest.builder(
                        instance, start, target.getPosition(),
                        actor.getBoundingBox(), profile)
                .maxPathLength(range).nodeSearchRange(range)
                .influences(influences).entityState(state).build();
        requestedTargetCell = TargetCell.of(target.getPosition());
        planning = system.sharedMesh().plan(SharedMeshRequest.builder(request).actor(actor.getUuid()).target(target.getUuid()).worldRevision(worldRevision).currentTick(currentTick).build());
        submittedTick = currentTick;
        consumed = false;
        replanRequested = false;
    }

    /**
     * Whether the search behind the current request may be replaced. Work in
     * flight is left alone until the tail the system observes says it
     * will never land, so replanning is paced by search completion and a
     * chased mob keeps receiving routes however slow searches become.
     */
    private boolean searchSettledOrAbandoned() {
        if (consumed) return true;
        long waited = currentTick - submittedTick;
        long minimumStallTicks = system.options().minimumSearchStallTicks();
        if (waited < minimumStallTicks) return false;
        long tail = system.metricsSnapshot().latency().p99Micros()
                / MICROS_PER_TICK;
        return waited >= Math.max(minimumStallTicks, tail * 4);
    }

    private void applyCompletedPlan() {
        consumed = true;
        try {
            NavigationPlan plan = planning.plan().join();
            if (!plan.usable()) {
                if (plan.status() == PathStatus.SHED) {
                    replanRequested = true;
                    earliestReplanTick =
                            currentTick + system.options().shedBackoffTicks();
                }
                return;
            }
            controller.follow(plan);
            following = true;
        } catch (IllegalArgumentException exception) {
            // The actor moved too far while an asynchronous plan was pending.
            replanRequested = true;
        } catch (CancellationException | CompletionException exception) {
            // A superseded or failed search publishes no route to follow.
        }
    }

    private boolean entitiesRemainCompatible() {
        if (closed || actor.isRemoved() || target.isRemoved()
                || isDead(actor) || isDead(target)) return false;
        Instance actorInstance = actor.getInstance();
        return actorInstance != null && actorInstance == target.getInstance();
    }

    private static boolean isDead(Entity entity) {
        return entity instanceof LivingEntity living && living.isDead();
    }
}
