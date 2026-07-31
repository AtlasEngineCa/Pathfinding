package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.metrics.NavigationMetricsSnapshot;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.adaptive.SharedMeshPursuit;
import ca.atlasengine.pathfinding.internal.movement.RouteSplicer;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.collision.CollisionUtils;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;
import net.minestom.server.coordinate.Vec;
import net.minestom.testing.Env;
import net.minestom.testing.EnvTest;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static ca.atlasengine.pathfinding.PursuitHarness.MOB_PER_TICK;
import static ca.atlasengine.pathfinding.PursuitHarness.SURFACE_Y;
import static ca.atlasengine.pathfinding.PursuitHarness.loadedGatedFlatInstance;
import static ca.atlasengine.pathfinding.PursuitHarness.orbit;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Route freshness, as distinct from liveness: how far behind the mob the route
 * it starts following actually begins.
 *
 * <p>A mob submits from where it is, but the plan lands a search later, by
 * which time the mob has moved. The adopted route therefore begins behind the
 * mob in proportion to search latency. This measures that gap directly, plus
 * the plans a follower refused outright as too stale to start.</p>
 */
@EnvTest
class RouteFreshnessTest {
    /** Simulated search latencies, in ticks; a tick is fifty milliseconds. */
    private static final int[] LATENCY_TICKS = {1, 3, 5, 10, 21, 28, 40};

    private static final int TICKS = 200;
    /** A latency past the follower's start tolerance, so splicing engages. */
    private static final int SPLICING_LATENCY_TICKS = 10;
    /**
     * The start tolerance {@code EntityNavigationController.follow} enforces
     * for a zombie footprint, and so the drift past which a plan computed from
     * the mob's current position no longer describes where it is.
     */
    private static final double FOLLOW_START_TOLERANCE = 1.5;
    /** Ticks a reversing target spends on each side of the mob. */
    private static final int REVERSE_PERIOD_TICKS = 25;

    @Test
    void controllerRouteFreshnessAcrossTheLatencySweep(Env env) {
        List<Freshness> sweep = new ArrayList<>();
        for (int latency : LATENCY_TICKS) sweep.add(pursueController(env, latency));
        String report = report("CONTROLLER ROUTE FRESHNESS", sweep);
        System.out.print(report);

        for (Freshness run : sweep) {
            assertTrue(run.adoptions() > 0,
                    () -> "no route was ever adopted at latency "
                            + run.latencyTicks() + report);
            // Without splicing this gap is the mob's own displacement during
            // the search, which passes the start tolerance at eight ticks and
            // reached 2.7 blocks at twenty-one. A spliced route begins on the
            // mob instead, however long the search took.
            if (!splicedRegime(run.latencyTicks())) continue;
            assertTrue(run.meanAdoptGap() <= FOLLOW_START_TOLERANCE,
                    () -> "routes adopted at latency " + run.latencyTicks()
                            + " began " + run.meanAdoptGap()
                            + " blocks from the mob" + report);
        }
    }

    @Test
    void adaptiveRouteFreshnessAcrossTheLatencySweep(Env env) {
        List<Freshness> sweep = new ArrayList<>();
        for (int latency : LATENCY_TICKS) sweep.add(pursueAdaptive(env, latency));
        String report = report("ADAPTIVE ROUTE FRESHNESS", sweep);
        System.out.print(report);

        for (Freshness run : sweep) {
            assertTrue(run.adoptions() > 0,
                    () -> "no route was ever adopted at latency "
                            + run.latencyTicks() + report);
            if (!splicedRegime(run.latencyTicks())) continue;
            assertTrue(run.meanAdoptGap() <= FOLLOW_START_TOLERANCE,
                    () -> "routes adopted at latency " + run.latencyTicks()
                            + " began " + run.meanAdoptGap()
                            + " blocks from the mob" + report);
            // A plan planned from a node the mob walks through is a plan the
            // follower can always start. Refusing most of them, as this path
            // did at half a second of latency, is the defect being fixed.
            assertTrue(run.stalePlans() * 2 <= run.completed(),
                    () -> "the follower refused " + run.stalePlans() + " of "
                            + run.completed() + " landed plans as too stale "
                            + "to start" + report);
        }
    }

    /**
     * Latencies past the follower's start tolerance, where a plan computed
     * from the mob's current position no longer lands close enough to it.
     */
    private static boolean splicedRegime(int latencyTicks) {
        return latencyTicks * MOB_PER_TICK > FOLLOW_START_TOLERANCE
                && latencyTicks <= 21;
    }

    /**
     * A spliced route is one route, not two laid end to end. Every route the
     * follower adopts while splicing is exercised must stay contiguous cell to
     * cell and sweepable by the entity's own bounding box, across the seam as
     * everywhere else.
     */
    @Test
    void everySplicedRouteIsContiguousAndSweepable(Env env) {
        PursuitHarness.SearchGate gate = new PursuitHarness.SearchGate();
        Instance instance = loadedGatedFlatInstance(env, gate);
        obstruct(instance);
        EntityCreature actor = new EntityCreature(EntityType.ZOMBIE);
        actor.setInstance(instance, new Pos(0.5, SURFACE_Y, 0.5)).join();
        for (int settle = 0; settle < 3; settle++) env.tick();

        NavigationSystem system = NavigationSystem.builder()
                .parallelism(4).queueCapacity(64)
                .movementPerTick(MOB_PER_TICK).build();
        EntityNavigationController controller = system.controller(actor);
        BoundingBox box = actor.getBoundingBox();
        List<List<PathNode>> adopted = new ArrayList<>();
        try {
            controller.moveTo(orbit(0));
            long submitted = system.metricsSnapshot().searches().submitted();
            long lastSubmitTick = 0;
            List<PathNode> route = controller.nodes();
            for (int tick = 1; tick <= TICKS; tick++) {
                if (tick - lastSubmitTick >= SPLICING_LATENCY_TICKS
                        && system.metricsSnapshot().searches().inFlight() > 0) {
                    gate.land(system);
                }
                controller.moveTo(orbit(tick));
                controller.tick();
                long now = system.metricsSnapshot().searches().submitted();
                if (now != submitted) {
                    submitted = now;
                    lastSubmitTick = tick;
                }
                List<PathNode> current = controller.nodes();
                if (current != route) {
                    route = current;
                    if (!current.isEmpty()) adopted.add(current);
                }
                env.tick();
            }
            RouteSplicer.Counts splices = controller.spliceCounts();
            assertTrue(splices.joined() > 0,
                    () -> "no route was spliced, so nothing was proved: "
                            + splices);
            assertEquals(0, splices.seamMismatch(),
                    () -> "a plan did not start at the seam it was asked "
                            + "to: " + splices);
            for (List<PathNode> spliced : adopted) {
                assertContiguous(spliced);
                assertSweepable(instance, box, spliced);
            }
        } finally {
            gate.openPermanently();
            controller.close();
            system.close();
            actor.remove();
            env.destroyInstance(instance);
        }
    }

    /**
     * A target that doubles back makes the route the mob is on worthless, and
     * splicing a fresh plan onto it would aim that plan from ground the mob
     * should not be covering. The direction test must refuse those, and the
     * mob must keep receiving routes anyway.
     */
    @Test
    void aReversingTargetRefusesSplicesAndStillKeepsRoutes(Env env) {
        PursuitHarness.SearchGate gate = new PursuitHarness.SearchGate();
        Instance instance = loadedGatedFlatInstance(env, gate);
        EntityCreature actor = new EntityCreature(EntityType.ZOMBIE);
        actor.setInstance(instance, new Pos(0.5, SURFACE_Y, 0.5)).join();
        for (int settle = 0; settle < 3; settle++) env.tick();

        NavigationSystem system = NavigationSystem.builder()
                .parallelism(4).queueCapacity(64)
                .movementPerTick(MOB_PER_TICK).build();
        EntityNavigationController controller = system.controller(actor);
        try {
            controller.moveTo(reversing(0));
            long submitted = system.metricsSnapshot().searches().submitted();
            long lastSubmitTick = 0;
            int ticksWithRoute = 0;
            for (int tick = 1; tick <= TICKS; tick++) {
                if (tick - lastSubmitTick >= SPLICING_LATENCY_TICKS
                        && system.metricsSnapshot().searches().inFlight() > 0) {
                    gate.land(system);
                }
                // The target crosses to the far side of the mob, so the route
                // the mob is on never points at it for long.
                controller.moveTo(reversing(tick));
                controller.tick();
                long now = system.metricsSnapshot().searches().submitted();
                if (now != submitted) {
                    submitted = now;
                    lastSubmitTick = tick;
                }
                env.tick();
                NavigationState state = controller.state();
                if (state == NavigationState.FOLLOWING
                        || state == NavigationState.PARTIAL) ticksWithRoute++;
            }
            RouteSplicer.Counts splices = controller.spliceCounts();
            assertTrue(splices.divergent() > 0,
                    () -> "a reversing target never refused a splice: "
                            + splices);
            assertEquals(0, splices.seamMismatch(),
                    () -> "a plan did not start at its seam: " + splices);
            int held = ticksWithRoute;
            assertTrue(held >= TICKS * 4 / 5,
                    () -> "a mob chasing a reversing target had no route on "
                            + (TICKS - held) + " of " + TICKS + " ticks");
        } finally {
            gate.openPermanently();
            controller.close();
            system.close();
            actor.remove();
            env.destroyInstance(instance);
        }
    }

    /** A target that teleports past the mob to the opposite side. */
    private static Pos reversing(int tick) {
        double x = tick / REVERSE_PERIOD_TICKS % 2 == 0 ? 15.5 : -15.5;
        return new Pos(x, SURFACE_Y, 0.5);
    }

    /** Pillars that force the chase onto bending, non-trivial routes. */
    private static void obstruct(Instance instance) {
        for (int x = -12; x <= 12; x += 4) {
            for (int z = -12; z <= 12; z += 4) {
                if (Math.abs(x) + Math.abs(z) < 4) continue;
                for (int y = 0; y < 3; y++) {
                    instance.setBlock(x, (int) SURFACE_Y + y, z, Block.STONE);
                }
            }
        }
    }

    private static void assertContiguous(List<PathNode> route) {
        for (int index = 1; index < route.size(); index++) {
            PathNode previous = route.get(index - 1);
            PathNode current = route.get(index);
            int stepX = Math.abs(current.graphX() - previous.graphX());
            int stepZ = Math.abs(current.graphZ() - previous.graphZ());
            assertTrue(stepX <= 1 && stepZ <= 1,
                    () -> "a route jumped cells at its seam: " + previous
                            + " -> " + current + " in " + route);
        }
    }

    private static void assertSweepable(
            Instance instance, BoundingBox box, List<PathNode> route) {
        for (int index = 1; index < route.size(); index++) {
            PathNode previous = route.get(index - 1);
            PathNode current = route.get(index);
            if (Math.abs(previous.y() - current.y()) > 1.0e-4) continue;
            Vec start = previous.asVec().add(0, 1.0e-5, 0);
            Vec end = current.asVec().add(0, 1.0e-5, 0);
            var physics = CollisionUtils.handlePhysics(instance, box,
                    start.asPos(), end.sub(start).asVec(), null, false);
            assertFalse(physics.collisionX() || physics.collisionY()
                            || physics.collisionZ(),
                    () -> "blocked segment " + previous + " -> " + current
                            + " in " + route);
        }
    }

    private static String report(String label, List<Freshness> sweep) {
        StringBuilder table = new StringBuilder(
                "\n===== " + label + " =====\n"
                        + "latency      adopt  meanGap   maxGap  stale  "
                        + "meanToTarget travelled  sub/done  "
                        + "armed/joined seam/over/exp/sup short/div\n");
        for (Freshness run : sweep) {
            RouteSplicer.Counts splices = run.splices();
            table.append(String.format(Locale.ROOT,
                    "%3dt/%4dms %6d %8.3f %8.3f %6d %13.3f %9.2f %5d/%-5d "
                            + "%6d/%-6d %3d/%3d/%3d/%3d %3d/%3d%n",
                    run.latencyTicks(), run.latencyTicks() * 50,
                    run.adoptions(), run.meanAdoptGap(), run.maxAdoptGap(),
                    run.stalePlans(), run.meanTargetDistance(),
                    run.travelled(), run.submitted(), run.completed(),
                    splices.armed(), splices.joined(), splices.seamMismatch(),
                    splices.overrun(), splices.expired(),
                    splices.superseded(), splices.shortRoute(),
                    splices.divergent()));
        }
        table.append("=".repeat(label.length() + 12)).append('\n');
        return table.toString();
    }

    /**
     * One pursuit run. {@code meanAdoptGap} is the distance from the mob to
     * the first node of each route it began following, averaged over the run:
     * how far behind the mob its own fresh route started.
     */
    private record Freshness(
            int latencyTicks, int adoptions, double meanAdoptGap,
            double maxAdoptGap, long stalePlans, double meanTargetDistance,
            double travelled, long submitted, long completed,
            RouteSplicer.Counts splices) {
    }

    private static Freshness pursueController(Env env, int latencyTicks) {
        PursuitHarness.SearchGate gate = new PursuitHarness.SearchGate();
        Instance instance = loadedGatedFlatInstance(env, gate);
        EntityCreature actor = new EntityCreature(EntityType.ZOMBIE);
        actor.setInstance(instance, new Pos(0.5, SURFACE_Y, 0.5)).join();
        for (int settle = 0; settle < 3; settle++) env.tick();

        NavigationSystem system = NavigationSystem.builder()
                .parallelism(4).queueCapacity(64)
                .movementPerTick(MOB_PER_TICK).build();
        EntityNavigationController controller = system.controller(actor);
        try {
            controller.moveTo(orbit(0));
            Sampler sampler = new Sampler(actor.getPosition());
            long submitted = system.metricsSnapshot().searches().submitted();
            long lastSubmitTick = 0;
            List<PathNode> route = controller.nodes();
            for (int tick = 1; tick <= TICKS; tick++) {
                if (tick - lastSubmitTick >= latencyTicks
                        && system.metricsSnapshot().searches().inFlight() > 0) {
                    gate.land(system);
                }
                Pos target = orbit(tick);
                controller.moveTo(target);
                controller.tick();
                long now = system.metricsSnapshot().searches().submitted();
                if (now != submitted) {
                    submitted = now;
                    lastSubmitTick = tick;
                }
                route = sampler.sample(
                        actor.getPosition(), target, route, controller.nodes());
                env.tick();
            }
            return sampler.freshness(latencyTicks, system.metricsSnapshot(),
                    controller.spliceCounts(), TICKS);
        } finally {
            gate.openPermanently();
            controller.close();
            system.close();
            actor.remove();
            env.destroyInstance(instance);
        }
    }

    private static Freshness pursueAdaptive(Env env, int latencyTicks) {
        PursuitHarness.SearchGate gate = new PursuitHarness.SearchGate();
        Instance instance = loadedGatedFlatInstance(env, gate);
        EntityCreature actor = new EntityCreature(EntityType.ZOMBIE);
        Entity target = new Entity(EntityType.ARMOR_STAND);
        target.setNoGravity(true);
        actor.setInstance(instance, new Pos(0.5, SURFACE_Y, 0.5)).join();
        target.setInstance(instance, orbit(0)).join();
        for (int settle = 0; settle < 3; settle++) env.tick();

        NavigationSystem system = NavigationSystem.builder()
                .parallelism(4).queueCapacity(64)
                .movementPerTick(MOB_PER_TICK).build();
        try {
            SharedMeshPursuit navigation =
                    system.sharedMesh().pursue(actor, target, 1, 0);
            Sampler sampler = new Sampler(actor.getPosition());
            long submitted = system.metricsSnapshot().searches().submitted();
            long lastSubmitTick = 0;
            List<PathNode> route = navigation.nodes();
            for (int tick = 1; tick <= TICKS; tick++) {
                target.teleport(orbit(tick));
                if (tick - lastSubmitTick >= latencyTicks
                        && system.metricsSnapshot().searches().inFlight() > 0) {
                    gate.land(system);
                }
                navigation.tick(1, tick);
                long now = system.metricsSnapshot().searches().submitted();
                if (now != submitted) {
                    submitted = now;
                    lastSubmitTick = tick;
                }
                route = sampler.sample(actor.getPosition(),
                        target.getPosition(), route, navigation.nodes());
                env.tick();
            }
            RouteSplicer.Counts splices = navigation.spliceCounts();
            navigation.close();
            return sampler.freshness(latencyTicks, system.metricsSnapshot(),
                    splices, TICKS);
        } finally {
            gate.openPermanently();
            system.close();
            actor.remove();
            target.remove();
            env.destroyInstance(instance);
        }
    }

    /** Accumulates the per-adoption gap and the per-tick pursuit quality. */
    private static final class Sampler {
        private int adoptions;
        private double totalGap;
        private double maximumGap;
        private double totalTargetDistance;
        private double travelled;
        private Point previous;

        private Sampler(Point start) {
            previous = start;
        }

        /** Returns the route to compare against on the following tick. */
        private List<PathNode> sample(Point position, Point target,
                                      List<PathNode> route,
                                      List<PathNode> current) {
            if (current != route && !current.isEmpty()) {
                double gap = position.distance(current.getFirst().asVec());
                adoptions++;
                totalGap += gap;
                maximumGap = Math.max(maximumGap, gap);
            }
            totalTargetDistance += position.distance(target);
            travelled += position.distance(previous);
            previous = position;
            return current;
        }

        private Freshness freshness(
                int latencyTicks, NavigationMetricsSnapshot end,
                RouteSplicer.Counts splices, int ticks) {
            return new Freshness(latencyTicks, adoptions,
                    adoptions == 0 ? 0 : totalGap / adoptions, maximumGap,
                    end.stalePlans(), totalTargetDistance / ticks, travelled,
                    end.searches().submitted(), end.searches().completed(),
                    splices);
        }
    }
}
