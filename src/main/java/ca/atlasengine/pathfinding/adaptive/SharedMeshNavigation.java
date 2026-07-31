package ca.atlasengine.pathfinding.adaptive;

import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.internal.adaptive.SharedMeshCoordinator;
import ca.atlasengine.pathfinding.profile.NavigationModifiers;
import ca.atlasengine.pathfinding.NavigationStrategy;
import ca.atlasengine.pathfinding.NavigationSystem;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import net.minestom.server.entity.Entity;
import net.minestom.server.instance.block.Block;

import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicLong;

/**
 * The shared-mesh planner of one {@link NavigationSystem}: the opted-in,
 * separately switchable alternative to individual A*.
 *
 * <p>Obtain it from {@link NavigationSystem#sharedMesh()}. It exists on every
 * system but plans individually until {@link
 * NavigationSystem.Builder#sharedMesh(SharedMeshOptions)} opts in, so mesh
 * behaviour is always a stated choice. {@link #disable()} takes it back out of
 * service at runtime without touching a single call site.</p>
 *
 * <p>Call {@link #tick(long)} once per owning world or server tick to apply
 * idle expiry and promotion hysteresis. Regions are managed through
 * {@link #forgetActor(Object)}, {@link #forgetTarget(Object)},
 * {@link #invalidateWorld(net.minestom.server.instance.block.Block.Getter,
 * long, long)} and {@link #unloadRegion(
 * net.minestom.server.instance.block.Block.Getter, int, int, int)}; what the
 * mesh retains is reported by {@link #status()}.</p>
 */
public final class SharedMeshNavigation implements AutoCloseable {
    private final NavigationSystem system;
    private final SharedMeshPolicy policy;
    private final AtomicLong meshRequests = new AtomicLong();
    private final AtomicLong sharedPlans = new AtomicLong();
    private final AtomicLong bypassedRequests = new AtomicLong();
    private volatile SharedMeshCoordinator coordinator;
    private volatile boolean enabled;
    private volatile boolean closed;

    public SharedMeshNavigation(
            NavigationSystem system, SharedMeshOptions options) {
        if (system == null || options == null) {
            throw new IllegalArgumentException("system and options");
        }
        this.system = system;
        policy = options.policy();
        enabled = options.enabled();
        coordinator = newCoordinator();
    }

    /**
     * Plans one request and automatically replaces any previous membership for
     * the same actor.
     *
     * <p>{@link SharedMeshHandle#source()} states what
     * answered it: only {@link SharedMeshSource#SHARED_TARGET_FIELD}
     * came from the mesh, and a {@link NavigationStrategy#PREFER_SHARED}
     * request falls back to an ordinary search silently and often. A request
     * that never consulted the mesh reports {@link
     * SharedMeshSource#UNSHARED_SEARCH}.</p>
     *
     * <p>Close the returned handle when the actor despawns, dies, changes
     * target, or no longer needs navigation.</p>
     */
    public SharedMeshHandle plan(SharedMeshRequest request) {
        if (request == null) throw new IllegalArgumentException("request");
        if (closed) throw new IllegalStateException("shared mesh is closed");
        SharedMeshCoordinator current = coordinator;
        if (!enabled
                || request.strategy() == NavigationStrategy.INDIVIDUAL_ONLY) {
            current.removeActor(request.actorKey());
            return unshared(request);
        }
        SharedMeshHandle handle;
        try {
            handle = current.request(
                    request.actorKey(), request.targetKey(),
                    request.worldRevision(), request.currentTick(),
                    request.search());
        } catch (IllegalStateException switchedOff) {
            // The kill switch closed this coordinator between the read above
            // and the call. A request that races it is planned individually.
            return unshared(request);
        }
        meshRequests.incrementAndGet();
        if (handle.source() == SharedMeshSource.SHARED_TARGET_FIELD) {
            sharedPlans.incrementAndGet();
        }
        return handle;
    }

    /**
     * Tracks one entity pursuing another through the mesh, including target
     * movement, membership replacement, shared-plan application, and cleanup.
     * A disabled mesh still returns a working pursuit that plans individually.
     */
    public SharedMeshPursuit pursue(
            Entity actor, Entity target, long worldRevision,
            long currentTick) {
        return pursue(actor, target, worldRevision, currentTick,
                List.of(), NavigationModifiers.NONE);
    }

    public SharedMeshPursuit pursue(
            Entity actor, Entity target, long worldRevision, long currentTick,
            List<NavigationInfluence> influences,
            NavigationModifiers modifiers) {
        return new SharedMeshPursuit(
                system, actor, target, worldRevision, currentTick,
                influences, modifiers);
    }

    /**
     * Applies idle target expiry, demotion hysteresis, and region cleanup.
     *
     * <p>{@code currentTick} is a tick <em>id</em>, not a timestamp: it is
     * compared against {@link SharedMeshPolicy#targetIdleTicks()} and
     * {@link SharedMeshPolicy#regionIdleTicks()}, and a pursuit swallows a
     * repeated id so a mob is never walked twice. This is not
     * Minestom's {@link net.minestom.server.Tickable}, whose argument is a
     * millisecond timestamp; feeding milliseconds here would expire retained
     * routes about fifty times too early and break that de-duplication.</p>
     */
    public void tick(long currentTick) {
        coordinator.tick(currentTick);
    }

    /** Releases one actor's membership immediately. */
    public void forgetActor(Object actorKey) {
        coordinator.removeActor(actorKey);
    }

    /** Releases every membership and target field of a despawned target. */
    public void forgetTarget(Object targetKey) {
        coordinator.removeTarget(targetKey);
    }

    /** Drops retained topology of one world after relevant block changes. */
    public void invalidateWorld(
            Block.Getter blocks, long newWorldRevision, long currentTick) {
        coordinator.invalidateWorld(blocks, newWorldRevision, currentTick);
    }

    /** Drops one unloaded regional cell, leaving other cells untouched. */
    public void unloadRegion(
            Block.Getter blocks, int regionX, int regionY, int regionZ) {
        coordinator.unloadRegion(blocks, regionX, regionY, regionZ);
    }

    /** Whether requests may consult the mesh at all. */
    public boolean enabled() {
        return enabled;
    }

    /**
     * Takes the mesh out of service at runtime. Every later request plans
     * individually whatever its strategy asks for, and every retained region,
     * target field, certificate, and membership is released at once; searches
     * the mesh had in flight are cancelled, so their callers see a cancelled
     * plan and replan normally. An operator can therefore switch off a
     * misbehaving mesh without changing or redeploying call sites.
     */
    public synchronized void disable() {
        if (closed || !enabled) return;
        enabled = false;
        SharedMeshCoordinator previous = coordinator;
        coordinator = newCoordinator();
        previous.close();
    }

    /** Puts the mesh back into service, cold: nothing is retained yet. */
    public synchronized void enable() {
        if (closed) throw new IllegalStateException("shared mesh is closed");
        enabled = true;
    }

    /** The promotion and lifecycle policy this mesh enforces. */
    public SharedMeshPolicy policy() {
        return policy;
    }

    public SharedMeshStatus status() {
        return new SharedMeshStatus(enabled, policy, coordinator.stats(),
                meshRequests.get(), sharedPlans.get(),
                bypassedRequests.get());
    }

    @Override
    public synchronized void close() {
        if (closed) return;
        closed = true;
        coordinator.close();
    }

    private SharedMeshHandle unshared(SharedMeshRequest request) {
        bypassedRequests.incrementAndGet();
        system.metrics().adaptiveDispatch(
                SharedMeshSource.UNSHARED_SEARCH);
        CompletableFuture<NavigationPlan> plan = system.planLatest(
                request.actorKey(), request.search());
        return new SharedMeshHandle(
                request.actorKey(), request.targetKey(),
                SharedMeshSource.UNSHARED_SEARCH, plan,
                () -> plan.cancel(true));
    }

    private SharedMeshCoordinator newCoordinator() {
        return new SharedMeshCoordinator(
                system::planLatest, policy, system.metrics());
    }
}
