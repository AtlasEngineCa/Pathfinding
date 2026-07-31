package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.metrics.NavigationMetricsSnapshot;import ca.atlasengine.pathfinding.adaptive.SharedMeshPursuit;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.Entity;
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

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * A mob chasing a moving target must keep receiving routes however far search
 * latency exceeds the time the target spends in one block cell.
 *
 * <p>Latency is expressed in whole ticks and enforced by holding every worker
 * inside {@link Instance#getBlock} until the tick loop lets it land, so a
 * search costs the same number of ticks on every machine and the run contains
 * no sleeps, no timeouts, and no wall-clock thresholds.</p>
 */
@EnvTest
class AdaptivePursuitLivenessTest {
    /** Simulated search latencies, in ticks; a tick is fifty milliseconds. */
    private static final int[] LATENCY_TICKS = {1, 2, 3, 5, 10};

    private static final int TICKS = 200;
    /**
     * The start tolerance {@code EntityNavigationController.follow} enforces
     * for a zombie footprint. A search longer than this many ticks lets the
     * mob outrun the position its own plan was computed from, which the
     * follower must keep refusing.
     */
    private static final double FOLLOW_START_TOLERANCE = 1.5;

    @Test
    void chasedMobKeepsRoutesWhileSearchOutlastsTargetCellDwell(Env env) {
        List<Pursuit> sweep = new ArrayList<>();
        for (int latency : LATENCY_TICKS) sweep.add(pursue(env, latency));

        StringBuilder table = new StringBuilder(
                "\n===== ADAPTIVE PURSUIT LIVENESS =====\n"
                        + "latency     ticksWithRoute routes travelled "
                        + "submitted/completed stale\n");
        for (Pursuit run : sweep) {
            table.append(String.format(Locale.ROOT,
                    "%3dt/%4dms  %6d/%d %6d %9.2f %10d/%-8d %5d%n",
                    run.latencyTicks(), run.latencyTicks() * 50,
                    run.ticksWithRoute(), TICKS, run.routesApplied(),
                    run.travelled(), run.submitted(), run.completed(),
                    run.stalePlans()));
        }
        table.append("=====================================\n");
        String report = table.toString();
        System.out.print(report);

        for (Pursuit run : sweep) {
            // Every submission but the one still running when the run ended
            // must reach the follower. A search abandoned because the target
            // stepped into the next cell is the defect this guards.
            assertTrue(run.completed() >= run.submitted() - 2,
                    () -> "only " + run.completed() + " of " + run.submitted()
                            + " searches survived to land" + report);
            assertTrue(run.ticksWithRoute() >= TICKS * 4 / 5,
                    () -> "a chased mob had no route to follow on "
                            + (TICKS - run.ticksWithRoute()) + " of " + TICKS
                            + " ticks" + report);
            assertTrue(run.travelled() >= 25.0,
                    () -> "a chased mob barely moved: " + run.travelled()
                            + " blocks" + report);
            if (run.latencyTicks() * MOB_PER_TICK > FOLLOW_START_TOLERANCE) {
                assertTrue(run.routesApplied() > 0,
                        () -> "a chased mob never followed a route" + report);
                continue;
            }
            assertTrue(run.routesApplied() >= run.completed() / 2,
                    () -> "only " + run.routesApplied() + " of "
                            + run.completed() + " landed plans became routes "
                            + "while the mob stayed inside the follower's "
                            + "start tolerance" + report);
        }
    }

    private record Pursuit(
            int latencyTicks, int ticksWithRoute, int routesApplied,
            double travelled, long submitted, long completed,
            long stalePlans) {
    }

    private static Pursuit pursue(Env env, int latencyTicks) {
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
            long submitted = system.metricsSnapshot().searches().submitted();
            long lastSubmitTick = 0;
            int ticksWithRoute = 0;
            int routesApplied = 0;
            int generation = navigation.generation();
            double travelled = 0;
            Point previous = actor.getPosition();
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
                env.tick();
                NavigationState state = navigation.state();
                if (state == NavigationState.FOLLOWING
                        || state == NavigationState.PARTIAL) ticksWithRoute++;
                if (navigation.generation() != generation) {
                    generation = navigation.generation();
                    routesApplied++;
                }
                Point position = actor.getPosition();
                travelled += position.distance(previous);
                previous = position;
            }
            navigation.close();
            NavigationMetricsSnapshot end = system.metricsSnapshot();
            return new Pursuit(latencyTicks, ticksWithRoute, routesApplied,
                    travelled, end.searches().submitted(),
                    end.searches().completed(), end.stalePlans());
        } finally {
            gate.openPermanently();
            system.close();
            actor.remove();
            target.remove();
            env.destroyInstance(instance);
        }
    }

}
