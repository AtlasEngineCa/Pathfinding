package ca.atlasengine.pathfinding;

import net.minestom.server.coordinate.ChunkRange;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.InstanceContainer;
import net.minestom.server.instance.block.Block;
import net.minestom.server.world.DimensionType;
import net.minestom.testing.Env;

import java.util.UUID;

/**
 * Shared rig for the pursuit suites: a flat world whose block reads suspend
 * every pathfinding worker until the tick loop releases them.
 *
 * <p>Latency is therefore expressed in whole ticks and costs the same number of
 * ticks on every machine, so a pursuit run contains no sleeps, no timeouts, and
 * no wall-clock thresholds.</p>
 */
final class PursuitHarness {
    /** A sprinting player, 5.6 blocks per second. */
    static final double SPRINT_PER_TICK = 0.28;
    static final double ORBIT_RADIUS = 10;
    static final double SURFACE_Y = 40;
    static final double MOB_PER_TICK = 0.2;

    private PursuitHarness() {
    }

    /** The pursued target's position, orbiting at a sprint. */
    static Pos orbit(int tick) {
        double angle = tick * SPRINT_PER_TICK / ORBIT_RADIUS;
        return new Pos(Math.cos(angle) * ORBIT_RADIUS + 0.5, SURFACE_Y,
                Math.sin(angle) * ORBIT_RADIUS + 0.5);
    }

    static Instance gatedFlatInstance(Env env, SearchGate gate) {
        InstanceContainer instance = new GatedInstance(env, gate);
        instance.setGenerator(unit ->
                unit.modifier().fillHeight(0, (int) SURFACE_Y, Block.STONE));
        env.process().instance().registerInstance(instance);
        return instance;
    }

    /** A gated flat world with its spawn-area chunks already resident. */
    static Instance loadedGatedFlatInstance(Env env, SearchGate gate) {
        Instance instance = gatedFlatInstance(env, gate);
        ChunkRange.chunksInRange(0, 0, 4,
                (x, z) -> instance.loadChunk(x, z).join());
        return instance;
    }

    private static final class GatedInstance extends InstanceContainer {
        private final SearchGate gate;

        private GatedInstance(Env env, SearchGate gate) {
            super(env.process(), UUID.randomUUID(), DimensionType.OVERWORLD,
                    null, DimensionType.OVERWORLD.key());
            this.gate = gate;
        }

        @Override
        public Block getBlock(
                int x, int y, int z, Block.Getter.Condition condition) {
            gate.awaitOnWorker();
            return super.getBlock(x, y, z, condition);
        }
    }

    /**
     * Suspends every pathfinding worker until the tick loop opens the gate,
     * then holds the loop until the released work has settled. A search
     * therefore takes exactly as many ticks as the loop makes it wait.
     */
    static final class SearchGate {
        private static final String WORKER_PREFIX =
                "minestom-entity-pathfinding-";

        private final Object lock = new Object();
        private boolean open;
        private int waiting;

        void awaitOnWorker() {
            if (!Thread.currentThread().getName()
                    .startsWith(WORKER_PREFIX)) return;
            synchronized (lock) {
                if (open) return;
                waiting++;
                boolean interrupted = false;
                try {
                    while (!open) {
                        try {
                            lock.wait();
                        } catch (InterruptedException exception) {
                            interrupted = true;
                        }
                    }
                } finally {
                    waiting--;
                    if (interrupted) Thread.currentThread().interrupt();
                }
            }
        }

        void land(NavigationSystem system) {
            synchronized (lock) {
                open = true;
                lock.notifyAll();
            }
            long deadline = System.nanoTime() + 30_000_000_000L;
            while (System.nanoTime() < deadline) {
                boolean idle;
                synchronized (lock) {
                    idle = waiting == 0;
                }
                if (idle && system.metricsSnapshot().searches().inFlight() == 0
                        && system.activeSearches() == 0
                        && system.queuedSearches() == 0) break;
                Thread.onSpinWait();
            }
            synchronized (lock) {
                open = false;
            }
        }

        void openPermanently() {
            synchronized (lock) {
                open = true;
                lock.notifyAll();
            }
        }
    }
}
