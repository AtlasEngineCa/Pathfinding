package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.NavigationRequest;import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;
import java.util.concurrent.CompletableFuture;

/**
 * Bounded, latest-per-actor admission layer for controller-owned searches.
 * Direct one-shot service submissions bypass it.
 */
final class ControllerPathScheduler implements AutoCloseable {
    private final AsyncEntityPathfindingService service;
    private final ControllerSchedulingOptions options;
    private final TreeSet<Task> waiting = new TreeSet<>((left, right) -> {
        int priority = Integer.compare(right.priority, left.priority);
        return priority != 0 ? priority : Long.compare(left.sequence, right.sequence);
    });
    private final Map<Object, Task> latest = new HashMap<>();
    private final ThreadLocal<ArrayDeque<Task>> dispatchQueue =
            ThreadLocal.withInitial(ArrayDeque::new);
    private final ThreadLocal<Boolean> dispatching =
            ThreadLocal.withInitial(() -> false);
    private long sequence;
    private int running;
    private boolean closed;

    ControllerPathScheduler(AsyncEntityPathfindingService service,
                            ControllerSchedulingOptions options) {
        this.service = service;
        this.options = options;
    }

    CompletableFuture<PathResult> submitLatest(
            Object key, NavigationRequest request, int priority) {
        if (key == null || request == null) {
            throw new IllegalArgumentException("key and request are required");
        }
        Task previous;
        Task evicted = null;
        Task task;
        boolean shed = false;
        List<Task> ready;
        synchronized (this) {
            if (closed) return CompletableFuture
                    .completedFuture(PathResult.SHED);
            previous = latest.remove(key);
            if (previous != null) waiting.remove(previous);

            task = new Task(this, key, request, priority, sequence++);
            if (waiting.size() >= options.maximumDeferredRequests()) {
                Task worst = waiting.isEmpty() ? null : waiting.last();
                if (worst == null
                        || waiting.comparator().compare(task, worst) >= 0) {
                    shed = true;
                } else {
                    waiting.remove(worst);
                    latest.remove(worst.key, worst);
                    evicted = worst;
                }
            }
            if (!shed) {
                latest.put(key, task);
                waiting.add(task);
            }
            ready = drain();
        }
        if (previous != null) cancelDetached(previous);
        if (evicted != null) evicted.complete(PathResult.SHED);
        if (shed) task.complete(PathResult.SHED);
        dispatch(ready);
        return task;
    }

    private List<Task> drain() {
        List<Task> ready = new ArrayList<>();
        while (!closed && running < options.maximumConcurrentSearches()
                && !waiting.isEmpty()) {
            Task task = waiting.pollFirst();
            if (task.isDone() || latest.get(task.key) != task) continue;
            running++;
            task.running = true;
            ready.add(task);
        }
        return ready;
    }

    private void dispatch(List<Task> ready) {
        if (ready.isEmpty()) return;
        ArrayDeque<Task> queue = dispatchQueue.get();
        queue.addAll(ready);
        if (dispatching.get()) return;
        dispatching.set(true);
        try {
            Task task;
            while ((task = queue.pollFirst()) != null) {
                Task current = task;
                CompletableFuture<PathResult> delegate =
                        service.submitLatest(current.key, current.request);
                current.delegate = delegate;
                delegate.whenComplete((result, failure) ->
                        finished(current, result, failure));
                if (current.isCancelled()) delegate.cancel(true);
            }
        } finally {
            queue.clear();
            dispatchQueue.remove();
            dispatching.remove();
        }
    }

    private void finished(
            Task task, PathResult result, Throwable failure) {
        List<Task> ready;
        synchronized (this) {
            if (task.running) {
                task.running = false;
                running--;
            }
            latest.remove(task.key, task);
            ready = drain();
        }
        if (!task.isDone()) {
            if (failure == null) task.complete(result);
            else task.completeExceptionally(failure);
        }
        dispatch(ready);
    }

    private void cancel(Task task) {
        synchronized (this) {
            waiting.remove(task);
            latest.remove(task.key, task);
        }
        cancelDetached(task);
    }

    private static void cancelDetached(Task task) {
        task.cancelDirect();
        if (task.delegate != null) task.delegate.cancel(true);
    }

    synchronized int deferredCount() {
        return waiting.size();
    }

    synchronized int runningCount() {
        return running;
    }

    @Override
    public void close() {
        ArrayList<Task> tasks;
        synchronized (this) {
            if (closed) return;
            closed = true;
            tasks = new ArrayList<>(latest.values());
            waiting.clear();
            latest.clear();
        }
        tasks.forEach(ControllerPathScheduler::cancelDetached);
    }

    private static final class Task
            extends CompletableFuture<PathResult> {
        private final ControllerPathScheduler owner;
        private final Object key;
        private final NavigationRequest request;
        private final int priority;
        private final long sequence;
        private volatile CompletableFuture<PathResult> delegate;
        private boolean running;

        private Task(ControllerPathScheduler owner, Object key,
                     NavigationRequest request, int priority, long sequence) {
            this.owner = owner;
            this.key = key;
            this.request = request;
            this.priority = priority;
            this.sequence = sequence;
        }

        @Override
        public boolean cancel(boolean mayInterruptIfRunning) {
            owner.cancel(this);
            return isCancelled();
        }

        private void cancelDirect() {
            if (isDone()) return;
            super.cancel(true);
        }
    }
}
