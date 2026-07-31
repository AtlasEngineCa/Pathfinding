package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.metrics.NavigationMetricsSnapshot;import ca.atlasengine.pathfinding.result.PathNode;import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
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
 * A caller that repeatedly points a controller at a moving target must keep
 * receiving routes however long a search takes, whether it retargets on every
 * tick or on a fixed interval.
 *
 * <p>Latency is expressed in whole ticks and enforced by holding every worker
 * inside {@link Instance#getBlock} until the tick loop lets it land, so a
 * search costs the same number of ticks on every machine and the run contains
 * no sleeps, no timeouts, and no wall-clock thresholds.</p>
 */
@EnvTest
class ControllerPursuitLivenessTest {
    /** Simulated search latencies, in ticks; a tick is fifty milliseconds. */
    private static final int[] LATENCY_TICKS = {1, 3, 5, 10, 21, 25, 40};

    private static final int TICKS = 200;
    /** Retarget cadence of the interval-driven caller, in ticks. */
    private static final int RETARGET_INTERVAL_TICKS = 20;
    /** Ticks a run may spend without a route before the first plan lands. */
    private static final int STARTUP_SLACK_TICKS = 5;
    /** Ticks a still-running search is left alone under repeated retargets. */
    private static final int PACED_TICKS = 30;
    /** Ticks after which a search that never lands must be abandoned. */
    private static final int ABANDON_TICKS = 65;

    @Test
    void perTickMoveToKeepsRoutesAtEverySearchLatency(Env env) {
        assertPursuitStaysAlive(env, 1, "PER-TICK moveTo");
    }

    @Test
    void intervalMoveToKeepsRoutesPastTheRecomputeInterval(Env env) {
        assertPursuitStaysAlive(env, RETARGET_INTERVAL_TICKS,
                RETARGET_INTERVAL_TICKS + "-TICK moveTo");
    }

    /**
     * A search that completed while a replan was queued must be applied before
     * the replan replaces it, or the work is discarded on arrival.
     */
    @Test
    void aLandedSearchIsReadBeforeAQueuedReplanSupersedesIt(Env env) {
        PursuitHarness.SearchGate gate = new PursuitHarness.SearchGate();
        Instance instance = loadedGatedFlatInstance(env, gate);
        EntityCreature actor = new EntityCreature(EntityType.ZOMBIE);
        actor.setInstance(instance, new Pos(0.5, SURFACE_Y, 0.5)).join();
        for (int settle = 0; settle < 3; settle++) env.tick();

        NavigationSystem system = NavigationSystem.builder()
                .parallelism(2).queueCapacity(16)
                .movementPerTick(MOB_PER_TICK).build();
        EntityNavigationController controller = system.controller(actor);
        try {
            controller.moveTo(new Pos(10.5, SURFACE_Y, 0.5));
            controller.tick();
            env.tick();
            controller.recomputePath();
            for (int wait = 0; wait < RETARGET_INTERVAL_TICKS; wait++) {
                controller.tick();
                env.tick();
            }
            controller.recomputePath();
            gate.land(system);
            controller.tick();

            assertFalse(controller.nodes().isEmpty(),
                    "a search that landed while a replan waited was discarded "
                            + "before the follower read it");
            assertTrue(controller.state() == NavigationState.FOLLOWING
                            || controller.state() == NavigationState.PARTIAL,
                    () -> "a landed route left the controller in "
                            + controller.state());
        } finally {
            gate.openPermanently();
            controller.close();
            system.close();
            actor.remove();
            env.destroyInstance(instance);
        }
    }

    /**
     * Pacing by completion must not become waiting forever: a search that
     * never lands is abandoned once the observed tail says it never will,
     * while a healthy one is never resubmitted just because ticks passed.
     */
    @Test
    void aSearchThatNeverLandsIsEventuallyAbandoned(Env env) {
        PursuitHarness.SearchGate gate = new PursuitHarness.SearchGate();
        Instance instance = loadedGatedFlatInstance(env, gate);
        EntityCreature actor = new EntityCreature(EntityType.ZOMBIE);
        actor.setInstance(instance, new Pos(0.5, SURFACE_Y, 0.5)).join();
        for (int settle = 0; settle < 3; settle++) env.tick();

        NavigationSystem system = NavigationSystem.builder()
                .parallelism(2).queueCapacity(16)
                .movementPerTick(MOB_PER_TICK).build();
        EntityNavigationController controller = system.controller(actor);
        try {
            Pos destination = new Pos(10.5, SURFACE_Y, 0.5);
            controller.moveTo(destination);
            for (int tick = 1; tick <= PACED_TICKS; tick++) {
                controller.moveTo(destination);
                controller.tick();
                env.tick();
            }
            assertEquals(1, system.metricsSnapshot().searches().submitted(),
                    "a retarget resubmitted a search that was still running");
            for (int tick = PACED_TICKS + 1; tick <= ABANDON_TICKS; tick++) {
                controller.moveTo(destination);
                controller.tick();
                env.tick();
            }
            assertTrue(system.metricsSnapshot().searches().submitted() >= 2,
                    "a search that never lands was never abandoned");
        } finally {
            gate.openPermanently();
            controller.close();
            system.close();
            actor.remove();
            env.destroyInstance(instance);
        }
    }

    private static void assertPursuitStaysAlive(
            Env env, int retargetTicks, String label) {
        List<Pursuit> sweep = new ArrayList<>();
        for (int latency : LATENCY_TICKS) {
            sweep.add(pursue(env, latency, retargetTicks));
        }

        StringBuilder table = new StringBuilder(
                "\n===== CONTROLLER PURSUIT LIVENESS: " + label + " =====\n"
                        + "latency     ticksWithRoute routes travelled "
                        + "submitted/completed superseded stale\n");
        for (Pursuit run : sweep) {
            table.append(String.format(Locale.ROOT,
                    "%3dt/%4dms  %6d/%d %6d %9.2f %10d/%-8d %8d %5d%n",
                    run.latencyTicks(), run.latencyTicks() * 50,
                    run.ticksWithRoute(), TICKS, run.routesApplied(),
                    run.travelled(), run.submitted(), run.completed(),
                    run.superseded(), run.stalePlans()));
        }
        table.append("==========================================\n");
        String report = table.toString();
        System.out.print(report);

        for (Pursuit run : sweep) {
            // Every submission but the one still running when the run ended
            // must reach the follower. A search abandoned because the caller
            // retargeted a moving mob is the defect this guards.
            assertTrue(run.completed() >= run.submitted() - 2,
                    () -> "only " + run.completed() + " of " + run.submitted()
                            + " searches survived to land" + report);
            int expected = TICKS - run.latencyTicks() - STARTUP_SLACK_TICKS;
            assertTrue(run.ticksWithRoute() >= expected,
                    () -> "a chased mob had no route to follow on "
                            + (TICKS - run.ticksWithRoute()) + " of " + TICKS
                            + " ticks" + report);
            assertTrue(run.routesApplied() >= run.completed() - 2,
                    () -> "only " + run.routesApplied() + " of "
                            + run.completed() + " landed searches became "
                            + "routes" + report);
            assertTrue(run.travelled() >= 25.0,
                    () -> "a chased mob barely moved: " + run.travelled()
                            + " blocks" + report);
        }
    }

    private record Pursuit(
            int latencyTicks, int ticksWithRoute, int routesApplied,
            double travelled, long submitted, long completed, long superseded,
            long stalePlans) {
    }

    private static Pursuit pursue(Env env, int latencyTicks, int retargetTicks) {
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
            long submitted = system.metricsSnapshot().searches().submitted();
            long lastSubmitTick = 0;
            int ticksWithRoute = 0;
            int routesApplied = 0;
            double travelled = 0;
            List<PathNode> route = controller.nodes();
            Point previous = actor.getPosition();
            for (int tick = 1; tick <= TICKS; tick++) {
                if (tick - lastSubmitTick >= latencyTicks
                        && system.metricsSnapshot().searches().inFlight() > 0) {
                    gate.land(system);
                }
                if (tick % retargetTicks == 0) controller.moveTo(orbit(tick));
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
                List<PathNode> current = controller.nodes();
                if (current != route) {
                    route = current;
                    if (!current.isEmpty()) routesApplied++;
                }
                Point position = actor.getPosition();
                travelled += position.distance(previous);
                previous = position;
            }
            NavigationMetricsSnapshot end = system.metricsSnapshot();
            return new Pursuit(latencyTicks, ticksWithRoute, routesApplied,
                    travelled, end.searches().submitted(),
                    end.searches().completed(), end.searches().superseded(),
                    end.stalePlans());
        } finally {
            gate.openPermanently();
            controller.close();
            system.close();
            actor.remove();
            env.destroyInstance(instance);
        }
    }

}
