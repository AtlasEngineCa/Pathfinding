package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.metrics.NavigationMetrics;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.CancellationSource;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Bounded asynchronous execution for all baseline navigation modes.
 *
 * <p>World reads happen on the configured worker threads. Callers must only
 * supply a {@link net.minestom.server.instance.block.Block.Getter} that is safe
 * to read concurrently. The queue is bounded rather than growing
 * memory without limit; a submission beyond the bound is shed, completing its
 * future normally with {@link PathResult#SHED}. Backpressure is routine, so it
 * is a value to branch on rather than an exception to catch.</p>
 *
 * <p>{@link #submitLatest(Object, NavigationRequest)} coalesces
 * replans for one entity: its previous running search is cooperatively
 * cancelled and its previous queued search is removed before the replacement
 * is submitted. A cancelled stale future can never publish a path result.</p>
 *
 * <p>Every search lifecycle transition is recorded into {@link #metrics()},
 * which an operator reads with {@link NavigationMetrics#snapshot()}. A search
 * increments its one terminal counter before it publishes its terminal future
 * state, so a caller that has observed the future has already observed the
 * counter.</p>
 */
public final class AsyncEntityPathfindingService implements AutoCloseable {
    private final EntityPathfinder pathfinding;
    private final ThreadPoolExecutor executor;
    private final NavigationMetrics metrics;
    private final int queueCapacity;
    private final Set<PathTask> tasks = ConcurrentHashMap.newKeySet();
    private final ConcurrentHashMap<Object, PathTask> latest = new ConcurrentHashMap<>();

    public AsyncEntityPathfindingService(int parallelism, int queueCapacity) {
        this(new EntityPathfinder(), parallelism, queueCapacity);
    }

    /** Records into a recorder shared with other services or coordinators. */
    public AsyncEntityPathfindingService(int parallelism, int queueCapacity,
                                          NavigationMetrics metrics) {
        this(new EntityPathfinder(), parallelism, queueCapacity, metrics);
    }

    public AsyncEntityPathfindingService(EntityPathfinder pathfinding,
                                          int parallelism,
                                          int queueCapacity) {
        this(pathfinding, parallelism, queueCapacity, new NavigationMetrics());
    }

    public AsyncEntityPathfindingService(EntityPathfinder pathfinding,
                                          int parallelism,
                                          int queueCapacity,
                                          NavigationMetrics metrics) {
        if (pathfinding == null) throw new IllegalArgumentException("pathfinding cannot be null");
        if (metrics == null) throw new IllegalArgumentException("metrics cannot be null");
        if (parallelism <= 0 || queueCapacity <= 0) {
            throw new IllegalArgumentException("parallelism and queueCapacity must be positive");
        }
        this.pathfinding = pathfinding;
        this.metrics = metrics;
        this.queueCapacity = queueCapacity;
        ThreadPoolExecutor pool = new ThreadPoolExecutor(
                parallelism,
                parallelism,
                0L,
                TimeUnit.MILLISECONDS,
                new ArrayBlockingQueue<>(queueCapacity),
                namedFactory(),
                new ThreadPoolExecutor.AbortPolicy()
        );
        this.executor = pool;
        metrics.bindQueueGauges(new NavigationMetrics.QueueGauges() {
            @Override
            public int activeSearches() {
                return pool.getActiveCount();
            }

            @Override
            public int queuedSearches() {
                return pool.getQueue().size();
            }

            @Override
            public int parallelism() {
                return pool.getCorePoolSize();
            }

            @Override
            public int queueCapacity() {
                return queueCapacity;
            }
        });
    }

    /** Cumulative search, adaptive, and controller telemetry. */
    public NavigationMetrics metrics() {
        return metrics;
    }

    public CompletableFuture<PathResult> submit(NavigationRequest request) {
        return submitInternal(null, request);
    }

    /**
     * Submits the newest replan for {@code key}, cancelling and evicting any
     * unfinished older replan for that key.
     */
    public CompletableFuture<PathResult> submitLatest(Object key,
                                                      NavigationRequest request) {
        if (key == null) throw new IllegalArgumentException("key cannot be null");
        return submitInternal(key, request);
    }

    private CompletableFuture<PathResult> submitInternal(Object key,
                                                         NavigationRequest request) {
        if (request == null) throw new IllegalArgumentException("request cannot be null");

        PathTask task = new PathTask(new CancellationSource(), metrics);
        metrics.searchSubmitted();
        tasks.add(task);

        Runnable command = () -> {
            if (task.isCancelled()) {
                finish(key, task);
                return;
            }
            metrics.searchStarted(System.nanoTime() - task.submittedNanos);
            try {
                PathResult result = pathfinding.findPath(
                        request,
                        new SearchControl(task.source.token(), Long.MAX_VALUE));
                if (task.settle()) {
                    metrics.searchCompleted(
                            System.nanoTime() - task.submittedNanos);
                    task.complete(result);
                }
            } catch (Throwable throwable) {
                if (task.settle()) {
                    metrics.searchFailed();
                    task.completeExceptionally(throwable);
                }
            } finally {
                finish(key, task);
            }
        };
        task.command = command;
        task.cancellationAction = () -> removeQueued(key, task);

        if (key == null) {
            execute(key, task, command);
        } else {
            // Publish and enqueue atomically with respect to other same-key
            // submissions. Otherwise a newer call can cancel this task after
            // publication but before its command exists in the queue.
            synchronized (latest) {
                PathTask previous = latest.put(key, task);
                if (previous != null) {
                    previous.superseded = true;
                    previous.cancel(true);
                }
                execute(key, task, command);
            }
        }
        return task;
    }

    private void execute(Object key, PathTask task, Runnable command) {
        try {
            executor.execute(command);
        } catch (RejectedExecutionException rejected) {
            finish(key, task);
            if (task.settle()) {
                metrics.searchRejected();
                task.complete(PathResult.SHED);
            }
        }
    }

    private void removeQueued(Object key, PathTask task) {
        Runnable command = task.command;
        if (command != null && executor.remove(command)) {
            finish(key, task);
        }
    }

    private void finish(Object key, PathTask task) {
        tasks.remove(task);
        if (key != null) latest.remove(key, task);
    }

    public int activeCount() {
        return executor.getActiveCount();
    }

    public int queuedCount() {
        return executor.getQueue().size();
    }

    public int parallelism() {
        return executor.getCorePoolSize();
    }

    /** The configured bound on {@link #queuedCount()}. */
    public int queueCapacity() {
        return queueCapacity;
    }

    @Override
    public void close() {
        tasks.forEach(task -> task.cancel(true));
        latest.clear();
        executor.shutdownNow();
        try {
            if (!executor.awaitTermination(5, TimeUnit.SECONDS)) {
                throw new IllegalStateException("Pathfinding workers did not terminate");
            }
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
        } finally {
            tasks.clear();
        }
    }

    private static ThreadFactory namedFactory() {
        AtomicInteger sequence = new AtomicInteger();
        return runnable -> {
            Thread thread = new Thread(
                    runnable,
                    "minestom-entity-pathfinding-" + sequence.incrementAndGet());
            thread.setDaemon(true);
            return thread;
        };
    }

    private static final class PathTask extends CompletableFuture<PathResult> {
        private final CancellationSource source;
        private final NavigationMetrics metrics;
        private final long submittedNanos = System.nanoTime();
        private final AtomicBoolean settled = new AtomicBoolean();
        private volatile Runnable command;
        private volatile Runnable cancellationAction = () -> { };
        private volatile boolean superseded;

        private PathTask(CancellationSource source, NavigationMetrics metrics) {
            this.source = source;
            this.metrics = metrics;
        }

        /**
         * Claims both the single terminal metric this search may report and
         * the right to publish its terminal future state, in that order. The
         * winner records before it publishes, so a caller that has seen the
         * future has already seen the counter.
         */
        private boolean settle() {
            return settled.compareAndSet(false, true);
        }

        @Override
        public boolean cancel(boolean mayInterruptIfRunning) {
            source.cancel();
            if (!settle()) return false;
            if (superseded) metrics.searchSuperseded();
            else metrics.searchCancelled();
            super.cancel(mayInterruptIfRunning);
            cancellationAction.run();
            return true;
        }
    }
}
