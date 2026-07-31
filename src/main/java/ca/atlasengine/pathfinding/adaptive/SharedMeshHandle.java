package ca.atlasengine.pathfinding.adaptive;

import ca.atlasengine.pathfinding.NavigationPlan;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;

/** Owned membership and result of one adaptive navigation request. */
public final class SharedMeshHandle implements AutoCloseable {
    private final Object actorKey;
    private final Object targetKey;
    private final SharedMeshSource source;
    private final CompletableFuture<NavigationPlan> plan;
    private final Runnable cleanup;
    private final AtomicBoolean closed = new AtomicBoolean();

    public SharedMeshHandle(
            Object actorKey, Object targetKey,
            SharedMeshSource source,
            CompletableFuture<NavigationPlan> plan, Runnable cleanup) {
        this.actorKey = actorKey;
        this.targetKey = targetKey;
        this.source = source;
        this.plan = plan;
        this.cleanup = cleanup;
    }

    public Object actorKey() {
        return actorKey;
    }

    public Object targetKey() {
        return targetKey;
    }

    public SharedMeshSource source() {
        return source;
    }

    public CompletableFuture<NavigationPlan> plan() {
        return plan;
    }

    public boolean closed() {
        return closed.get();
    }

    @Override
    public void close() {
        if (closed.compareAndSet(false, true)) cleanup.run();
    }
}
