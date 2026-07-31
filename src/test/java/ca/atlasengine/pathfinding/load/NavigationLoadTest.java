package ca.atlasengine.pathfinding.load;

import ca.atlasengine.pathfinding.AsyncEntityPathfindingService;import ca.atlasengine.pathfinding.adaptive.SharedMeshNavigation;
import ca.atlasengine.pathfinding.adaptive.SharedMeshRetention;
import ca.atlasengine.pathfinding.adaptive.SharedMeshRequest;
import ca.atlasengine.pathfinding.metrics.NavigationMetricsSnapshot;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.NavigationState;
import ca.atlasengine.pathfinding.NavigationSystem;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.TestWorld;
import ca.atlasengine.pathfinding.adaptive.SharedMeshPolicy;
import ca.atlasengine.pathfinding.internal.adaptive.SharedMeshCoordinator;
import ca.atlasengine.pathfinding.adaptive.SharedMeshHandle;
import ca.atlasengine.pathfinding.adaptive.SharedMeshSource;
import ca.atlasengine.pathfinding.NavigationPlanCache;
import ca.atlasengine.pathfinding.adaptive.SharedMeshOptions;
import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;
import net.minestom.testing.Env;
import net.minestom.testing.EnvTest;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Random;
import java.util.concurrent.CompletableFuture;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Sustained MMORPG-scale soak over the real {@link NavigationSystem}.
 *
 * <p>Everything is derived from {@code -Dpathfinding.loadSeed}; the printed
 * banner repeats the exact command that replays a failure. Scale knobs are
 * {@code -Dpathfinding.loadEntities}, {@code -Dpathfinding.loadTicks},
 * {@code -Dpathfinding.loadParallelism} and
 * {@code -Dpathfinding.loadQueueCapacity}.</p>
 *
 * <p>The loop is paced to a fifty-millisecond tick by {@link LoadClock}, so
 * every invariant expressed per tick means what it says on a server rather
 * than on this JVM. {@code -Dpathfinding.loadTickMillis=0} frees the loop
 * again and is only useful for showing what the pacing removes.</p>
 *
 * <p>Not covered: chunk load/unload, network, and real server tick contention.
 * The run is one JVM against one in-memory instance whose chunks are all
 * resident, so it exercises search, planning, and follow behaviour only.</p>
 */
@EnvTest
class NavigationLoadTest {
    private static final String SEED_PROPERTY = "pathfinding.loadSeed";
    private static final String ENTITY_PROPERTY = "pathfinding.loadEntities";
    private static final String TICK_PROPERTY = "pathfinding.loadTicks";
    private static final String PARALLELISM_PROPERTY =
            "pathfinding.loadParallelism";
    private static final String QUEUE_PROPERTY =
            "pathfinding.loadQueueCapacity";
    private static final String TICK_MILLIS_PROPERTY =
            "pathfinding.loadTickMillis";

    private static final long DEFAULT_SEED = 0x10AD_5EEDL;
    private static final int DEFAULT_ENTITIES = 160;
    /** Paced, so the default run costs {@code ticks * 50ms} of wall clock. */
    private static final int DEFAULT_TICKS = 200;
    private static final int DEFAULT_QUEUE_CAPACITY = 256;
    private static final int DEFAULT_TICK_MILLIS = 50;

    private static final int ARENA_HALF = 56;

    @Test
    void sustainedMixedTrafficHoldsInvariants(Env env) {
        long seed = Long.getLong(SEED_PROPERTY, DEFAULT_SEED);
        int entities = Integer.getInteger(ENTITY_PROPERTY, DEFAULT_ENTITIES);
        int ticks = Integer.getInteger(TICK_PROPERTY, DEFAULT_TICKS);
        int queueCapacity = Integer.getInteger(
                QUEUE_PROPERTY, DEFAULT_QUEUE_CAPACITY);
        int parallelism = Integer.getInteger(PARALLELISM_PROPERTY,
                Math.max(1, Math.min(8,
                        Runtime.getRuntime().availableProcessors() / 2)));
        int tickMillis = Integer.getInteger(
                TICK_MILLIS_PROPERTY, DEFAULT_TICK_MILLIS);
        System.out.printf(Locale.ROOT,
                "%nnavigation load run: replay with -D%s=%d -D%s=%d -D%s=%d "
                        + "-D%s=%d -D%s=%d -D%s=%d%n",
                SEED_PROPERTY, seed, ENTITY_PROPERTY, entities,
                TICK_PROPERTY, ticks, PARALLELISM_PROPERTY, parallelism,
                QUEUE_PROPERTY, queueCapacity,
                TICK_MILLIS_PROPERTY, tickMillis);

        Random random = new Random(seed);
        Instance instance = env.createFlatInstance();
        LoadArena arena = LoadArena.build(instance, ARENA_HALF, random);

        LoadMetrics metrics = new LoadMetrics();
        int cacheCapacity = Math.max(16, entities / 2);
        NavigationPlanCache<LoadActor.CacheKey> cache =
                new NavigationPlanCache<>(cacheCapacity);

        try (NavigationSystem system = NavigationSystem.builder()
                .parallelism(parallelism)
                .queueCapacity(queueCapacity)
                .sharedMesh(SharedMeshOptions.ENABLED)
                .build()) {
            List<Entity> roamers = spawnRoamers(instance, arena, entities);
            List<LoadActor> actors = spawnActors(
                    system, metrics, cache, instance, arena, roamers,
                    entities, random);
            SharedMeshNavigation coordinator = system.sharedMesh();
            SharedMeshPolicy config = coordinator.policy();
            int sampleInterval = Math.max(5, ticks / 60);
            int warmupTicks = Math.min(60, ticks / 8);
            int unloadInterval = Math.max(50, ticks / 3);
            long worldRevision = 1;
            LoadClock clock = LoadClock.ofMillis(tickMillis);
            long start = System.nanoTime();
            NavigationMetricsSnapshot warm = system.metricsSnapshot();
            clock.start();

            for (int tick = 0; tick < ticks; tick++) {
                long tickStart = System.nanoTime();
                if (tick == ticks * 3 / 5) {
                    arena.editTerrain();
                    worldRevision++;
                    coordinator.invalidateWorld(instance, worldRevision, tick);
                }
                if (tick > 0 && tick % unloadInterval == 0) {
                    coordinator.unloadRegion(instance,
                            (tick / unloadInterval) % 2 == 0 ? -1 : 0, 0, -1);
                }
                moveRoamers(roamers, arena, tick);
                for (LoadActor actor : actors) actor.tick(tick, worldRevision);
                long adaptiveStart = System.nanoTime();
                system.sharedMesh().tick(tick);
                long envStart = System.nanoTime();
                env.tick();
                long now = System.nanoTime();
                metrics.coordinatorTickTook(envStart - adaptiveStart);
                metrics.envTickTook(now - envStart);
                metrics.tickTook(now - tickStart);
                if (tick == warmupTicks) warm = system.metricsSnapshot();
                if (tick % sampleInterval == 0) {
                    metrics.sample(sample(tick, coordinator, system, cache));
                }
                metrics.tickPeriodTook(clock.awaitTick() - tickStart);
            }
            NavigationMetricsSnapshot trafficEnd = system.metricsSnapshot();

            for (LoadActor actor : actors) actor.close();
            for (Entity roamer : roamers) {
                coordinator.forgetTarget(roamer.getUuid());
                roamer.remove();
            }
            quiesce(system);
            long drainStart = ticks;
            long drainEnd = ticks + config.regionIdleTicks() + 200;
            for (long tick = drainStart; tick <= drainEnd; tick++) {
                system.sharedMesh().tick(tick);
                if (tick % sampleInterval == 0) {
                    metrics.sample(sample(tick, coordinator, system, cache));
                }
            }
            double wall = (System.nanoTime() - start) / 1e9;
            NavigationMetricsSnapshot end = system.metricsSnapshot();

            long heapBefore = usedHeap();
            System.gc();
            System.gc();
            long heapAfter = usedHeap();
            metrics.printSummary("SOAK", seed, entities, ticks, wall,
                    heapBefore, heapAfter, queueCapacity, clock, warm, end);

            // A loop that outran its clock would put every per-tick invariant
            // back on JVM time, which is the confound the pacing removes.
            double median = metrics.medianTickPeriodMillis();
            assertTrue(!clock.paced()
                            || median >= clock.periodMillis() * 0.9,
                    () -> "the loop ran faster than the "
                            + clock.periodMillis() + "ms tick it models: "
                            + "median period " + median + "ms");
            assertInvariants(metrics, config, entities, parallelism,
                    queueCapacity, seed, warm, trafficEnd, end);
            assertGroundActorsTravelled(actors, seed);

            SharedMeshRetention drained =
                    coordinator.status().regions();
            assertEquals(0, drained.actors(),
                    () -> "membership leak after every handle closed: "
                            + drained);
            assertEquals(0, drained.regions(),
                    () -> "regions never expired after traffic stopped: "
                            + drained);
            assertEquals(0, drained.targets(),
                    () -> "target fields survived region expiry: " + drained);
            assertEquals(0, drained.retainedNodes(),
                    () -> "retained mesh survived region expiry: " + drained);
            assertEquals(0, drained.observedPlans(),
                    () -> "observed plans survived region expiry: " + drained);
            assertEquals(0, drained.certifiedPlans(),
                    () -> "certified plans survived region expiry: " + drained);
            assertEquals(0, drained.publishedPlans(),
                    () -> "published plans survived region expiry: " + drained);
        }
    }

    private void assertInvariants(
            LoadMetrics metrics, SharedMeshPolicy config,
            int entities, int parallelism, int queueCapacity, long seed,
            NavigationMetricsSnapshot warm,
            NavigationMetricsSnapshot trafficEnd,
            NavigationMetricsSnapshot end) {
        String replay = " (replay with -D" + SEED_PROPERTY + "=" + seed + ")";
        NavigationMetricsSnapshot.SearchCounts searches = end.searches();

        assertEquals(0, metrics.unexpectedCount(),
                () -> "search futures failed outside the documented contract"
                        + replay + ": " + describe(metrics, end));

        // The library's own dispatch total must agree with the number of calls
        // the harness made. A parallel count that only agreed with itself
        // would prove nothing about the surface an operator reads.
        assertEquals(metrics.adaptiveCalls(), end.adaptiveSources().total(),
                () -> "adaptive dispatch counter disagrees with the number of "
                        + "adaptivePlan() calls the harness made" + replay);

        // After every handle and controller closed and the pool drained, no
        // search may remain in flight. A search that never settles is a
        // follower that never navigates again.
        assertEquals(0, searches.inFlight(),
                () -> "searches never settled after the pool drained" + replay
                        + ": " + describe(metrics, end));
        assertEquals(searches.submitted(), searches.terminated(),
                () -> "search lifecycle counters do not balance" + replay
                        + ": " + describe(metrics, end));

        for (LoadMetrics.Sample sample : metrics.samples()) {
            // Documented snapshot consistency, on every reading.
            assertTrue(sample.submitted() >= sample.terminated(),
                    () -> "snapshot reported more terminals than submissions"
                            + replay + ": " + sample);
            assertTrue(sample.latencyCount() <= sample.submitted(),
                    () -> "latency samples outnumber submitted searches"
                            + replay + ": " + sample);
            assertTrue(sample.regions() <= config.maximumRegions(),
                    () -> "region cap exceeded" + replay + ": " + sample);
            assertTrue(sample.targets()
                            <= sample.regions() * config.maximumTargetsPerRegion(),
                    () -> "target cap exceeded" + replay + ": " + sample);
            assertTrue(sample.retainedNodes()
                            <= sample.regions() * config.maximumNodesPerRegion(),
                    () -> "retained-node cap exceeded" + replay + ": " + sample);
            // retainedNodes counts mesh endpoints only, so a region's three
            // plan maps could leak without moving it. Observation is capped
            // per region by the promotion threshold; certification and
            // publication cannot outnumber the source/target pairs the node
            // cap admits.
            assertTrue(sample.observedPlans()
                            <= sample.regions() * config.promotionRequests(),
                    () -> "observed-plan map exceeded the promotion threshold"
                            + replay + ": " + sample);
            assertTrue(sample.retainedPlans() <= sample.regions()
                            * config.maximumNodesPerRegion(),
                    () -> "retained plan maps grew past the per-region node "
                            + "budget" + replay + ": " + sample);
            assertTrue(sample.publishedPlans() <= sample.certifiedPlans(),
                    () -> "a plan was published without a certificate"
                            + replay + ": " + sample);
            assertEquals(queueCapacity, sample.queueCapacity(),
                    () -> "configured queue capacity is not observable"
                            + replay + ": " + sample);
            assertTrue(sample.coordinatorActors() <= entities,
                    () -> "coordinator tracks more actors than exist"
                            + replay + ": " + sample);
            assertTrue(sample.queued() <= queueCapacity,
                    () -> "bounded queue overflowed" + replay + ": " + sample);
            assertEquals(parallelism, sample.parallelism(),
                    () -> "the fixed pool changed size" + replay + ": "
                            + sample);
            assertTrue(sample.pathfindingThreads() <= parallelism,
                    () -> "worker pool grew past its fixed size"
                            + replay + ": " + sample);
            assertTrue(sample.planCache() <= Math.max(16, entities / 2),
                    () -> "plan cache exceeded its capacity"
                            + replay + ": " + sample);
        }

        assertTrue(searches.submitted() >= entities,
                () -> "the run performed almost no work" + replay + ": "
                        + describe(metrics, end));

        // The steady window must not be a slow start followed by a stall: the
        // library must still have been dispatching after warmup.
        assertTrue(trafficEnd.adaptiveSources().total()
                        > warm.adaptiveSources().total(),
                () -> "no adaptive dispatches after warmup" + replay);

        NavigationMetricsSnapshot.ControllerOutcomeCounts outcomes =
                end.controllerOutcomes();
        assertTrue(outcomes.total() >= entities,
                () -> "no navigations finished" + replay + ": "
                        + describe(metrics, end));
        assertTrue(outcomes.travelling() <= outcomes.total(),
                () -> "more navigations travelled than finished" + replay
                        + ": " + describe(metrics, end));
        // The rate an operator alarms on is the travel-filtered one. Its
        // denominator only ever shrinks against the same stalls, so it can
        // never read below the diluted lifetime rate; a wiring that made it
        // read lower would be hiding stalls.
        assertTrue(outcomes.travelStallRate() >= outcomes.stallRate(),
                () -> "travel-filtered stall rate "
                        + outcomes.travelStallRate() + " read below the "
                        + "diluted rate " + outcomes.stallRate() + replay
                        + ": " + describe(metrics, end));
        assertTrue(outcomes.travelStallRate() <= STUCK_BUDGET,
                () -> "controller travel stall rate "
                        + outcomes.travelStallRate() + " exceeded the budget "
                        + STUCK_BUDGET + replay + ": "
                        + describe(metrics, end));
        // Independently computed by the harness from its own submission
        // distances, so agreement is evidence and not a tautology.
        assertTrue(metrics.travelStuckFraction() <= STUCK_BUDGET,
                () -> "harness travel-only stall rate "
                        + metrics.travelStuckFraction() + " exceeded the "
                        + "budget " + STUCK_BUDGET + replay + ": "
                        + describe(metrics, end));
        assertTrue(end.stalePlans() <= metrics.stalePlans(),
                () -> "the library counted more stale hand-offs than the "
                        + "follower refused" + replay + ": "
                        + describe(metrics, end));
    }

    /**
     * Ground families on connected terrain must actually move. The arena is a
     * four-block lattice, so every free cell is reachable, patrol legs are
     * twelve to twenty blocks long, and the slowest ground patrol covers about
     * thirty blocks over the paced default. An actor below this bound never
     * navigated at all, which is what a starved pool now looks like: at
     * {@code parallelism=1, queueCapacity=32} with 320 entities the patrols
     * that never receive a route sit at exactly zero.
     */
    private static void assertGroundActorsTravelled(
            List<LoadActor> actors, long seed) {
        List<LoadActor> grounded = actors.stream()
                .filter(actor -> actor.shape() == LoadActor.Shape.PATROL)
                .filter(actor -> GROUND_FAMILIES.contains(
                        actor.entity().getEntityType()))
                .toList();
        assertTrue(grounded.size() >= 4,
                "the fixture stopped producing ground patrols");
        double[] travelled = grounded.stream()
                .mapToDouble(LoadActor::travelled).sorted().toArray();
        System.out.printf(Locale.ROOT,
                "ground patrols n=%d travelled min=%.1f p50=%.1f max=%.1f "
                        + "(floor %.1f)%n",
                travelled.length, travelled[0],
                travelled[travelled.length / 2],
                travelled[travelled.length - 1], MINIMUM_TRAVEL);
        List<LoadActor> pinned = grounded.stream()
                .filter(actor -> actor.travelled() < MINIMUM_TRAVEL).toList();
        assertTrue(pinned.isEmpty(),
                () -> "ground patrols never moved (replay with -D"
                        + SEED_PROPERTY + "=" + seed + "): "
                        + pinned.stream().map(actor ->
                                actor.entity().getEntityType().key().value()
                                        + "@" + actor.entity().getPosition()
                                        + " travelled="
                                        + actor.travelled()
                                        + " state=" + actor.state()).toList());
    }

    private static final List<EntityType> GROUND_FAMILIES = List.of(
            EntityType.ZOMBIE, EntityType.SILVERFISH, EntityType.SPIDER);

    /** Blocks a ground patrol must cover before it counts as having moved. */
    private static final double MINIMUM_TRAVEL = 4.0;

    /**
     * Fraction of finished navigations allowed to end STUCK.
     *
     * <p>Observed 0.0000 across sixteen seeds at 160 entities and at 480
     * entities over 1800 ticks, so this is not the observed number rounded up.
     * It is set by the negative control instead: starving the pool to
     * {@code parallelism=1, queueCapacity=32} with 320 entities sheds 79% of
     * searches and drives the travel-only rate to 0.046. Two percent sits an
     * order of magnitude below that broken configuration and well above the
     * mob-on-mob contention a healthy run produces, so it separates the two
     * without being fitted to either.</p>
     *
     * <p>That control only reaches 0.046 while the loop free-runs
     * ({@code -Dpathfinding.loadTickMillis=0}), because starvation is a
     * shortage of searches per tick and a free-running tick is a few
     * milliseconds long. On the fifty-millisecond clock the same starvation
     * cannot drive the rate past 0.007 at any density the arena holds: one
     * worker still lands nineteen searches a tick at eight times the entity
     * count, where the free-running loop left it 0.9. Starvation there
     * surfaces as patrols that never receive a route, which
     * {@link #assertGroundActorsTravelled} catches. The budget is kept as the
     * operator-facing bound it always was rather than refitted to the paced
     * numbers.</p>
     */
    private static final double STUCK_BUDGET = 0.02;

    private static String describe(
            LoadMetrics metrics, NavigationMetricsSnapshot end) {
        StringBuilder text = new StringBuilder();
        text.append(end.searches())
                .append(' ').append(end.adaptiveSources())
                .append(' ').append(end.controllerOutcomes())
                .append(" handleCancels=").append(metrics.handleCancellations())
                .append(" handleShed=").append(metrics.handleShedPlans())
                .append(" stale=").append(metrics.stalePlans())
                .append(" libraryStale=").append(end.stalePlans())
                .append(" travelEpisodes=").append(metrics.travelEpisodes())
                .append(" travelStuck=").append(
                        metrics.travelTerminals(NavigationState.STUCK));
        for (Throwable failure : metrics.unexpectedSamples()) {
            text.append("\n  unexpected: ").append(failure);
            StackTraceElement[] trace = failure.getStackTrace();
            for (int index = 0; index < Math.min(6, trace.length); index++) {
                text.append("\n      at ").append(trace[index]);
            }
        }
        return text.toString();
    }

    private static LoadMetrics.Sample sample(
            long tick, SharedMeshNavigation coordinator,
            NavigationSystem system, NavigationPlanCache<?> cache) {
        SharedMeshRetention stats =
                coordinator.status().regions();
        int total = 0;
        int pathfinding = 0;
        for (Thread thread : Thread.getAllStackTraces().keySet()) {
            total++;
            if (thread.getName().startsWith("minestom-entity-pathfinding-")
                    || thread.getName().startsWith("minestom-pathfinding-")) {
                pathfinding++;
            }
        }
        NavigationMetricsSnapshot snapshot = system.metricsSnapshot();
        return new LoadMetrics.Sample(tick, stats.regions(),
                stats.promotedRegions(), stats.actors(), stats.targets(),
                stats.retainedNodes(), stats.observedPlans(),
                stats.certifiedPlans(), stats.publishedPlans(), cache.size(),
                snapshot.queue().queuedSearches(),
                snapshot.queue().activeSearches(),
                snapshot.queue().parallelism(),
                snapshot.queue().queueCapacity(), total, pathfinding,
                snapshot.searches().submitted(),
                snapshot.searches().terminated(),
                snapshot.latency().count(),
                snapshot.adaptiveSources().total());
    }

    private static void quiesce(NavigationSystem system) {
        long deadline = System.nanoTime() + 30_000_000_000L;
        while ((system.activeSearches() > 0 || system.queuedSearches() > 0)
                && System.nanoTime() < deadline) {
            Thread.onSpinWait();
        }
        assertEquals(0, system.queuedSearches(),
                "queued searches never drained after traffic stopped");
    }

    private static long usedHeap() {
        Runtime runtime = Runtime.getRuntime();
        return runtime.totalMemory() - runtime.freeMemory();
    }

    private static List<Entity> spawnRoamers(
            Instance instance, LoadArena arena, int entities) {
        int packs = Math.max(1, entities / 40);
        List<Entity> roamers = new ArrayList<>(packs);
        for (int index = 0; index < packs; index++) {
            Entity roamer = new Entity(EntityType.ARMOR_STAND);
            roamer.setNoGravity(true);
            double angle = 2 * Math.PI * index / packs;
            roamer.setInstance(instance, roamerPosition(arena, angle)).join();
            roamers.add(roamer);
        }
        return roamers;
    }

    private static Pos roamerPosition(LoadArena arena, double angle) {
        double radius = arena.half() * 0.45;
        return new Pos(Math.cos(angle) * radius + 0.5, LoadArena.SURFACE_Y,
                Math.sin(angle) * radius + 0.5);
    }

    private static void moveRoamers(
            List<Entity> roamers, LoadArena arena, int tick) {
        for (int index = 0; index < roamers.size(); index++) {
            double angle = 2 * Math.PI * index / roamers.size()
                    + tick * 0.006;
            roamers.get(index).teleport(roamerPosition(arena, angle));
        }
    }

    private static List<LoadActor> spawnActors(
            NavigationSystem system, LoadMetrics metrics,
            NavigationPlanCache<LoadActor.CacheKey> cache, Instance instance,
            LoadArena arena, List<Entity> roamers, int entities,
            Random random) {
        EntityType[] families = {
                EntityType.ZOMBIE, EntityType.SPIDER, EntityType.SILVERFISH,
                EntityType.BEE, EntityType.AXOLOTL};
        List<Pos> cells = freeCells(arena);
        java.util.Collections.shuffle(cells, random);
        List<LoadActor> actors = new ArrayList<>(entities);
        Object objectiveKey = "objective";
        for (int index = 0; index < entities; index++) {
            EntityType type = families[index % families.length];
            LoadActor.Shape shape = index % 5 < 2 ? LoadActor.Shape.PATROL
                    : index % 5 < 4 ? LoadActor.Shape.CONVERGE
                    : LoadActor.Shape.PURSUIT;
            Pos home = homeFor(type, arena, cells, index, shape, random);
            EntityCreature mob = new EntityCreature(type);
            mob.setInstance(instance, home).join();

            Pos[] waypoints;
            Object[] keys;
            int replan;
            if (shape == LoadActor.Shape.PATROL) {
                Pos far = patrolPartner(home, arena, random);
                assertTrue(far.distance(home) >= MINIMUM_PATROL_LEG,
                        () -> "fixture produced a degenerate patrol leg: "
                                + home + " -> " + far);
                waypoints = new Pos[]{home, far};
                keys = new Object[]{
                        "patrol:" + cellKey(home), "patrol:" + cellKey(far)};
                replan = 40;
            } else if (shape == LoadActor.Shape.CONVERGE) {
                waypoints = new Pos[]{arena.objective()};
                keys = new Object[]{objectiveKey};
                replan = 30;
            } else {
                waypoints = new Pos[]{arena.objective()};
                keys = new Object[]{objectiveKey};
                replan = 18;
            }
            Entity roamer = roamers.get(index % roamers.size());
            actors.add(new LoadActor(system, metrics, cache, mob, shape,
                    waypoints, keys, () -> roamer, replan,
                    random.nextInt(7)));
        }
        return actors;
    }

    private static Pos homeFor(
            EntityType type, LoadArena arena, List<Pos> cells, int index,
            LoadActor.Shape shape, Random random) {
        if (type == EntityType.AXOLOTL) {
            Pos pond = arena.pondCentre();
            return pond.add(random.nextInt(7) - 3, 0, random.nextInt(7) - 3);
        }
        Pos cell = cells.get(index % cells.size());
        if (type == EntityType.BEE) return cell.add(0, 4, 0);
        if (shape == LoadActor.Shape.CONVERGE) {
            double scale = 36.0 / Math.max(1, arena.half());
            Pos scaled = new Pos(cell.x() * scale, cell.y(), cell.z() * scale);
            return arena.inPond(scaled.x(), scaled.z()) ? cell : scaled;
        }
        return cell;
    }

    /**
     * A patrol leg always steps toward the arena centre, so an edge spawn can
     * never clamp its far waypoint back onto its own cell.
     */
    private static Pos patrolPartner(Pos home, LoadArena arena, Random random) {
        for (int attempt = 0; attempt < 8; attempt++) {
            int axes = random.nextInt(3);
            double x = axes == 1 ? home.x()
                    : home.x() + inward(home.x(), random);
            double z = axes == 0 ? home.z()
                    : home.z() + inward(home.z(), random);
            Pos partner = new Pos(x, home.y(), z);
            if (!arena.inPond(x, z)
                    && partner.distance(home) >= MINIMUM_PATROL_LEG) {
                return partner;
            }
        }
        return new Pos(home.x() + inward(home.x(), random), home.y(),
                home.z() + inward(home.z(), random));
    }

    private static final double MINIMUM_PATROL_LEG = 8;

    private static double inward(double coordinate, Random random) {
        double step = (random.nextInt(3) + 3) * 4;
        return coordinate > 0 ? -step : step;
    }

    private static String cellKey(Pos position) {
        return position.blockX() + "/" + position.blockZ();
    }

    /**
     * Cells offset from the pillar lattice, so no spawn starts inside stone,
     * and clear of the pond, which belongs to the amphibious family.
     */
    private static List<Pos> freeCells(LoadArena arena) {
        List<Pos> cells = new ArrayList<>();
        for (int x = -arena.half() + 2; x <= arena.half(); x += 4) {
            for (int z = -arena.half() + 2; z <= arena.half(); z += 4) {
                if (arena.inPond(x + 0.5, z + 0.5)) continue;
                cells.add(new Pos(x + 0.5, LoadArena.SURFACE_Y, z + 0.5));
            }
        }
        return cells;
    }

    @Test
    void deliberateSaturationRejectsWithoutUnboundedGrowth() {
        int parallelism = 2;
        int queueCapacity = 16;
        int submissions = queueCapacity * 80;
        TestWorld world = new TestWorld()
                .floor(-80, 80, -80, 80, 39, Block.STONE);
        List<CompletableFuture<PathResult>> futures =
                new ArrayList<>(submissions);
        int peakQueued = 0;
        int peakOutstanding = 0;
        try (NavigationSystem system = NavigationSystem.builder()
                .parallelism(parallelism)
                .queueCapacity(queueCapacity)
                .build()) {
            NavigationRequest request = NavigationRequest.builder(
                            world, new Pos(0.5, 40, 0.5),
                            new Pos(70.5, 40, 70.5),
                            new BoundingBox(0.6, 1.95, 0.6),
                            BuiltinNavigationProfiles.forEntityType(
                                    EntityType.ZOMBIE))
                    .maxPathLength(160).nodeSearchRange(160).build();
            for (int index = 0; index < submissions; index++) {
                futures.add(system.submit(request));
                peakQueued = Math.max(peakQueued, system.queuedSearches());
                if (index % 64 == 0) {
                    int outstanding = 0;
                    for (CompletableFuture<PathResult> future : futures) {
                        if (!future.isDone()) outstanding++;
                    }
                    peakOutstanding = Math.max(peakOutstanding, outstanding);
                }
            }

            int shed = 0;
            int accepted = 0;
            for (CompletableFuture<PathResult> future : futures) {
                PathResult result = future.join();
                if (result.shed()) shed++;
                else accepted++;
            }
            NavigationMetricsSnapshot end = system.metricsSnapshot();
            NavigationMetricsSnapshot.SearchCounts counts = end.searches();
            System.out.printf(Locale.ROOT,
                    "%n===== NAVIGATION LOAD SATURATION =====%n"
                            + "submitted=%d accepted=%d shedFutures=%d "
                            + "peakQueued=%d queueCapacity=%d reported=%d "
                            + "peakOutstanding=%d parallelism=%d%n"
                            + "metrics %s%n"
                            + "======================================%n",
                    submissions, accepted, shed, peakQueued,
                    queueCapacity, end.queue().queueCapacity(),
                    peakOutstanding, parallelism, counts);

            int observedQueued = peakQueued;
            int observedOutstanding = peakOutstanding;
            assertEquals(submissions, accepted + shed);
            assertTrue(observedQueued <= queueCapacity,
                    () -> "queue grew past its bound: " + observedQueued);
            assertTrue(observedOutstanding <= queueCapacity + parallelism,
                    () -> "outstanding work grew past queue+workers: "
                            + observedOutstanding);

            // The builder value must be readable from the snapshot alone, or
            // nothing can chart queue depth against the bound it approaches.
            assertEquals(queueCapacity, end.queue().queueCapacity(),
                    () -> "configured queue capacity is not observable: "
                            + end.queue());
            assertEquals(queueCapacity, system.options().queueCapacity());

            // The documented bounded shed mode, read from the surface an
            // operator would alert on rather than inferred from exceptions.
            assertTrue(counts.rejected() > 0,
                    () -> "saturation never reached the bound: " + counts);
            assertEquals(shed, counts.rejected(),
                    () -> "the shed counter disagrees with the futures that "
                            + "actually came back shed: " + counts);
            assertEquals(submissions, counts.submitted(),
                    () -> "submission counter missed work: " + counts);
            assertEquals(counts.submitted(), counts.terminated(),
                    () -> "saturated searches did not all settle: " + counts);
            assertEquals(0, counts.failed(),
                    () -> "saturation produced a non-shed failure: " + counts);
            assertEquals(0, counts.cancelled(),
                    () -> "saturation cancelled work it should have shed: "
                            + counts);
        }
    }

    @Test
    void regionCapacityBindsAndFallsBackToUntrackedSearches() {
        SharedMeshPolicy config = SharedMeshPolicy.builder()
                .maximumRegions(2)
                .maximumTargetsPerRegion(2)
                .promotionRequests(4)
                .retentionRequests(1)
                .targetIdleTicks(4)
                .regionIdleTicks(8)
                .build();
        TestWorld world = new TestWorld()
                .floor(-400, 400, -8, 8, 39, Block.STONE);
        int peakRegions = 0;
        int untracked = 0;
        try (NavigationSystem system = NavigationSystem.builder()
                .parallelism(2).queueCapacity(512)
                .sharedMesh(SharedMeshOptions.enabledWith(config)).build()) {
            SharedMeshNavigation coordinator = system.sharedMesh();
            List<SharedMeshHandle> open = new ArrayList<>();
            for (int tick = 0; tick < 64; tick++) {
                for (int region = 0; region < 6; region++) {
                    double x = region * 64 + 0.5;
                    NavigationRequest request = NavigationRequest.builder(
                                    world, new Pos(x, 40, 0.5),
                                    new Pos(x + 6, 40, 0.5),
                                    new BoundingBox(0.6, 1.95, 0.6),
                                    BuiltinNavigationProfiles.forEntityType(
                                            EntityType.ZOMBIE))
                            .maxPathLength(32).nodeSearchRange(32).build();
                    SharedMeshHandle handle = system.sharedMesh().plan(SharedMeshRequest.builder(request).actor("actor-" + region + "-" + tick).target("target-" + region).worldRevision(1).currentTick(tick).build());
                    if (handle.source()
                            == SharedMeshSource.UNTRACKED_FALLBACK) {
                        untracked++;
                    }
                    open.add(handle);
                }
                system.sharedMesh().tick(tick);
                peakRegions = Math.max(peakRegions,
                        coordinator.status().regions().regions());
                if (open.size() > 24) {
                    open.removeFirst().close();
                }
            }
            open.forEach(SharedMeshHandle::close);
            for (int tick = 64; tick < 200; tick++) {
                system.sharedMesh().tick(tick);
            }
            System.out.printf(Locale.ROOT,
                    "%n===== NAVIGATION LOAD REGION CAP =====%n"
                            + "peakRegions=%d cap=%d untrackedFallbacks=%d "
                            + "drained=%s%n"
                            + "======================================%n",
                    peakRegions, config.maximumRegions(), untracked,
                    coordinator.status().regions());
            int observedRegions = peakRegions;
            assertTrue(observedRegions <= config.maximumRegions(),
                    () -> "region cap exceeded: " + observedRegions);
            assertTrue(untracked > 0,
                    "an oversubscribed coordinator must fall back to A*");
            assertEquals(0, coordinator.status().regions().regions(),
                    () -> "regions survived idle expiry: "
                            + coordinator.status().regions());
        }
    }

    /**
     * A server closes its {@link NavigationSystem} while mobs are still
     * planning. One outstanding adaptive search is enough to reproduce.
     */
    @Test
    void closeWithPendingSearchesTerminatesWorkers() {
        TestWorld world = new TestWorld()
                .floor(-256, 256, -256, 256, 39, Block.STONE);
        NavigationSystem system = NavigationSystem.builder()
                .parallelism(1).queueCapacity(256)
                .sharedMesh(SharedMeshOptions.ENABLED).build();
        NavigationRequest request = NavigationRequest.builder(
                        world, new Pos(0.5, 40, 0.5),
                        new Pos(200.5, 40, 200.5),
                        new BoundingBox(0.6, 1.95, 0.6),
                        BuiltinNavigationProfiles.forEntityType(
                                EntityType.ZOMBIE))
                .maxPathLength(512).nodeSearchRange(512).build();
        for (int index = 0; index < 8; index++) {
            system.sharedMesh().plan(SharedMeshRequest.builder(request).actor("actor-" + index).target("target").worldRevision(1).currentTick(index).build())
                    .plan().exceptionally(failure -> null);
        }
        assertTrue(livePathfindingThreads() > 0,
                "the pool never started a worker");

        RuntimeException closeFailure = null;
        try {
            system.close();
        } catch (RuntimeException failure) {
            closeFailure = failure;
            // SharedMeshCoordinator.close() already flipped its closed
            // flag, so a second call skips it and finally shuts the pool down.
            // Without this the leaked workers would follow the whole JVM.
            system.close();
        }
        if (closeFailure != null) {
            throw new AssertionError(
                    "NavigationSystem.close() threw while adaptive searches "
                            + "were pending, so AsyncEntityPathfindingService"
                            + ".close() never ran and the bounded worker pool "
                            + "was leaked", closeFailure);
        }

        long deadline = System.nanoTime() + 10_000_000_000L;
        while (livePathfindingThreads() > 0 && System.nanoTime() < deadline) {
            Thread.onSpinWait();
        }
        assertEquals(0, livePathfindingThreads(),
                () -> "workers survived close(): " + threadNames());
        for (Thread thread : Thread.getAllStackTraces().keySet()) {
            assertFalse(!thread.isDaemon()
                            && thread.getName().contains("pathfinding"),
                    () -> "close() left a non-daemon thread: "
                            + thread.getName());
        }
    }

    private static int livePathfindingThreads() {
        int count = 0;
        for (Thread thread : Thread.getAllStackTraces().keySet()) {
            if (thread.isAlive() && (thread.getName().startsWith(
                    "minestom-entity-pathfinding-")
                    || thread.getName().startsWith("minestom-pathfinding-"))) {
                count++;
            }
        }
        return count;
    }

    private static List<String> threadNames() {
        return Thread.getAllStackTraces().keySet().stream()
                .map(Thread::getName)
                .filter(name -> name.contains("pathfinding")).toList();
    }
}
