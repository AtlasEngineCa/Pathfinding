package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.event.NavigationCancelledEvent;
import ca.atlasengine.pathfinding.event.NavigationCompletedEvent;
import ca.atlasengine.pathfinding.event.NavigationEvent;
import ca.atlasengine.pathfinding.event.PathComputedEvent;
import ca.atlasengine.pathfinding.event.RouteReplanEvent;
import ca.atlasengine.pathfinding.result.PathStatus;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;
import net.minestom.server.event.EventNode;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;
import net.minestom.testing.Env;
import net.minestom.testing.EnvTest;
import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

/**
 * The event surface. Events reach an {@link EventNode} on the ticking thread,
 * and a system nobody listens to never builds one.
 */
@EnvTest
class NavigationEventTest {
    @Test
    void eventsReachTheNodeOnTheTickingThread(Env env) {
        Instance instance = TestWorlds.loadedFlat(env, 1);
        EntityCreature mob = TestWorlds.spawn(instance, EntityType.ZOMBIE,
                new Pos(0.5, 40, 0.5));
        env.tick();

        List<NavigationEvent> seen = new CopyOnWriteArrayList<>();
        List<Thread> threads = new CopyOnWriteArrayList<>();
        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            EventNode<NavigationEvent> node = navigation.eventNode();
            assertNotNull(node, "a system must expose an event node");
            node.addListener(PathComputedEvent.class, e -> {
                seen.add(e);
                threads.add(Thread.currentThread());
            });
            node.addListener(NavigationCompletedEvent.class, seen::add);
            node.addListener(RouteReplanEvent.class, seen::add);

            EntityNavigationController controller = navigation.controller(mob);
            controller.moveTo(new Pos(3.5, 40, 0.5));

            Thread ticking = Thread.currentThread();
            long deadline = System.nanoTime() + Duration.ofSeconds(10).toNanos();
            while (seen.stream().noneMatch(
                    e -> e instanceof NavigationCompletedEvent)) {
                if (System.nanoTime() > deadline) {
                    fail("no completion event; saw " + seen);
                }
                controller.tick();
                env.tick();
            }

            assertTrue(seen.stream().anyMatch(e -> e instanceof RouteReplanEvent),
                    "asking for a new target announces a replan");
            PathComputedEvent computed = (PathComputedEvent) seen.stream()
                    .filter(e -> e instanceof PathComputedEvent)
                    .findFirst().orElseThrow();
            assertEquals(PathStatus.FOUND, computed.status());
            assertSame(mob, computed.getEntity(),
                    "an entity event carries the entity it is about");
            assertTrue(computed.nodes().size() > 1, "a route was announced");
            assertEquals(List.of(ticking), List.copyOf(threads),
                    "events are called on the thread that ticked");
            controller.close();
        } finally {
            mob.remove();
        }
    }

    @Test
    void anOutcomeIsAnnouncedOnceRatherThanEveryTickItPersists(Env env) {
        Instance instance = TestWorlds.loadedFlat(env, 1);
        EntityCreature mob = TestWorlds.spawn(instance, EntityType.ZOMBIE,
                new Pos(0.5, 40, 0.5));
        env.tick();

        List<NavigationEvent> arrivals = new CopyOnWriteArrayList<>();
        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            navigation.eventNode()
                    .addListener(NavigationCompletedEvent.class, arrivals::add);
            EntityNavigationController controller = navigation.controller(mob);
            controller.moveTo(new Pos(3.5, 40, 0.5));

            TestWorlds.tickUntil(env, controller, () -> !arrivals.isEmpty(),
                    () -> "never arrived: " + controller.state());
            // COMPLETED persists, so a state-shaped implementation would
            // announce it again on every one of these ticks.
            for (int i = 0; i < 20; i++) {
                controller.tick();
                env.tick();
            }
            assertEquals(1, arrivals.size(),
                    "arrival is a transition, not a state");
            controller.close();
        } finally {
            mob.remove();
        }
    }

    @Test
    void watchedBlockChangesReachEveryControllerTheSystemOwns(Env env) {
        Instance instance = TestWorlds.loadedFlat(env, 1);
        EntityCreature mob = TestWorlds.spawn(instance, EntityType.ZOMBIE,
                new Pos(0.5, 40, 0.5));
        env.tick();

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            navigation.watchBlockChanges(instance);
            EntityNavigationController controller = navigation.controller(mob);
            controller.moveTo(new Pos(12.5, 40, 0.5));

            TestWorlds.awaitRoute(env, controller);
            int before = controller.generation();

            for (int y = 40; y < 43; y++) instance.setBlock(6, y, 0, Block.STONE);
            env.tick();

            TestWorlds.tickUntil(env, controller,
                    () -> controller.generation() != before,
                    () -> "a watched block change did not replan the route");
            assertTrue(controller.generation() > before,
                    "the route was replanned after the world changed");
            controller.close();
        } finally {
            mob.remove();
        }
    }

    @Test
    void aBlockChangeIsAnnouncedAsItsOwnReplanReason(Env env) {
        Instance instance = TestWorlds.loadedFlat(env, 1);
        EntityCreature mob = TestWorlds.spawn(instance, EntityType.ZOMBIE,
                new Pos(0.5, 40, 0.5));
        env.tick();

        List<RouteReplanEvent.Reason> reasons = new ArrayList<>();
        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            navigation.eventNode().addListener(RouteReplanEvent.class,
                    e -> reasons.add(e.reason()));
            navigation.watchBlockChanges(instance);
            EntityNavigationController controller = navigation.controller(mob);
            controller.moveTo(new Pos(12.5, 40, 0.5));

            TestWorlds.awaitRoute(env, controller);
            assertTrue(reasons.contains(RouteReplanEvent.Reason.NEW_TARGET),
                    "the first plan came from a new target, saw " + reasons);

            for (int y = 40; y < 43; y++) instance.setBlock(6, y, 0, Block.STONE);
            env.tick();

            TestWorlds.tickUntil(env, controller,
                    () -> reasons.contains(RouteReplanEvent.Reason.BLOCK_CHANGE),
                    () -> "a block change was not announced; saw " + reasons);
            controller.close();
        } finally {
            mob.remove();
        }
    }

    @Test
    void aSystemNobodyListensToBuildsNoEvents(Env env) {
        Instance instance = TestWorlds.loadedFlat(env, 1);
        EntityCreature mob = TestWorlds.spawn(instance, EntityType.ZOMBIE,
                new Pos(0.5, 40, 0.5));
        env.tick();

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            // eventNode() is never called, so no node exists to emit into.
            EntityNavigationController controller = navigation.controller(mob);
            controller.moveTo(new Pos(3.5, 40, 0.5));
            TestWorlds.tickUntil(env, controller,
                    () -> controller.state() == NavigationState.COMPLETED,
                    () -> "never arrived: " + controller.state());
            // Navigating without an event node must behave identically.
            assertEquals(NavigationState.COMPLETED, controller.state());
            controller.close();
        } finally {
            mob.remove();
        }
    }

    @Test
    void aNodeAskedForLateStillReceivesEvents(Env env) {
        Instance instance = TestWorlds.loadedFlat(env, 1);
        EntityCreature mob = TestWorlds.spawn(instance, EntityType.ZOMBIE,
                new Pos(0.5, 40, 0.5));
        env.tick();

        List<NavigationEvent> seen = new CopyOnWriteArrayList<>();
        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            // Controller created before anyone asked for the node.
            EntityNavigationController controller = navigation.controller(mob);
            navigation.eventNode()
                    .addListener(NavigationCompletedEvent.class, seen::add);

            controller.moveTo(new Pos(3.5, 40, 0.5));
            TestWorlds.tickUntil(env, controller, () -> !seen.isEmpty(),
                    () -> "a controller created before the node was ignored");
            controller.close();
        } finally {
            mob.remove();
        }
    }

    @Test
    void closingAFinishedNavigationKeepsItsOutcome(Env env) {
        Instance instance = TestWorlds.loadedFlat(env, 1);
        EntityCreature mob = TestWorlds.spawn(instance, EntityType.ZOMBIE,
                new Pos(0.5, 40, 0.5));
        env.tick();

        List<NavigationEvent> cancellations = new CopyOnWriteArrayList<>();
        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            navigation.eventNode().addListener(
                    NavigationCancelledEvent.class, cancellations::add);
            EntityNavigationController controller = navigation.controller(mob);
            controller.moveTo(new Pos(3.5, 40, 0.5));
            TestWorlds.tickUntil(env, controller,
                    () -> controller.state() == NavigationState.COMPLETED,
                    () -> "never arrived: " + controller.state());

            controller.close();
            assertEquals(NavigationState.COMPLETED, controller.state(),
                    "closing does not rewrite how the navigation ended");
            assertTrue(cancellations.isEmpty(),
                    "a completed navigation is not also cancelled");
        } finally {
            mob.remove();
        }
    }

    @Test
    void closingANavigationInProgressCancelsItOnce(Env env) {
        Instance instance = TestWorlds.loadedFlat(env, 1);
        EntityCreature mob = TestWorlds.spawn(instance, EntityType.ZOMBIE,
                new Pos(0.5, 40, 0.5));
        env.tick();

        List<NavigationEvent> cancellations = new CopyOnWriteArrayList<>();
        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            navigation.eventNode().addListener(
                    NavigationCancelledEvent.class, cancellations::add);
            EntityNavigationController controller = navigation.controller(mob);
            controller.moveTo(new Pos(12.5, 40, 0.5));
            TestWorlds.awaitRoute(env, controller);

            controller.close();
            controller.close();
            assertEquals(NavigationState.CANCELLED, controller.state());
            assertEquals(1, cancellations.size(),
                    "one navigation reports one outcome");
        } finally {
            mob.remove();
        }
    }
}
