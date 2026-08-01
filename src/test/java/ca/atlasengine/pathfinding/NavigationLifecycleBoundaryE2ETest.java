package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.event.RouteReplanEvent;
import ca.atlasengine.pathfinding.adaptive.SharedMeshOptions;
import ca.atlasengine.pathfinding.adaptive.SharedMeshPursuit;
import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.result.PathStatus;
import net.minestom.server.coordinate.ChunkRange;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;
import net.minestom.server.world.DimensionType;
import net.minestom.testing.Env;
import net.minestom.testing.EnvTest;
import net.kyori.adventure.key.Key;
import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

/**
 * End-to-end contracts at boundaries that ordinary positive-coordinate,
 * permanently-loaded arenas do not exercise.
 */
@EnvTest
class NavigationLifecycleBoundaryE2ETest {
    private static final double SPEED = 0.18;

    @Test
    void groundNavigationCrossesFromNegativeToPositiveChunks(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 2,
                (x, z) -> instance.loadChunk(x, z).join());
        EntityCreature mob = spawn(instance, new Pos(-6.5, 40, -0.5));

        try (var service = new AsyncEntityPathfindingService(1, 8)) {
            EntityNavigationController controller = controller(mob, service);
            controller.moveTo(new Pos(6.5, 40, 0.5));

            runUntilTerminal(env, controller, 600);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    "coordinate and chunk sign changes must not split the route");
            assertFalse(controller.nodes().isEmpty(),
                    "the completed crossing must have produced a route");
            controller.close();
        } finally {
            if (!mob.isRemoved()) mob.remove();
        }
    }

    @Test
    void staleRouteBlockedByWallGetsOneSearchAndRecovers(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 2,
                (x, z) -> instance.loadChunk(x, z).join());
        EntityCreature mob = spawn(instance, new Pos(0.5, 40, 0.5));
        for (int y = 40; y <= 42; y++) {
            instance.setBlock(4, y, 0, Block.STONE);
        }

        List<RouteReplanEvent.Reason> replans = new ArrayList<>();
        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            navigation.eventNode().addListener(RouteReplanEvent.class,
                    event -> replans.add(event.reason()));
            EntityNavigationController controller = navigation.controller(mob);
            Pos target = new Pos(8.5, 40, 0.5);
            List<PathNode> staleStraightRoute = new ArrayList<>();
            for (int x = 0; x <= 8; x++) {
                staleStraightRoute.add(new PathNode(x + 0.5, 40, 0.5,
                        PathNode.Movement.WALK));
            }
            controller.follow(new NavigationPlan(mob.getPosition(), target,
                    mob.getBoundingBox(),
                    BuiltinNavigationProfiles.forEntity(mob), PathStatus.FOUND,
                    staleStraightRoute, staleStraightRoute.size(), 0));

            runUntilTerminal(env, controller, 900);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    "a timed-out cached waypoint should trigger a real detour");
            assertEquals(List.of(RouteReplanEvent.Reason.REQUESTED), replans,
                    "waypoint recovery must submit exactly one search");
            controller.close();
        } finally {
            if (!mob.isRemoved()) mob.remove();
        }
    }

    @Test
    void chunkUnloadRemovingFollowerDoesNotCrashControllerTick(Env env) {
        Instance instance = env.createFlatInstance();
        instance.loadChunk(0, 0).join();
        instance.loadChunk(1, 0).join();
        EntityCreature mob = spawn(instance, new Pos(2.5, 40, 2.5));

        try (var service = new AsyncEntityPathfindingService(1, 8)) {
            EntityNavigationController controller = controller(mob, service);
            controller.moveTo(new Pos(20.5, 40, 2.5));
            for (int tick = 0; tick < 10; tick++) {
                controller.tick();
                env.tick();
            }

            instance.unloadChunk(0, 0);
            env.tick();

            assertDoesNotThrow(controller::tick,
                    "a normal Minestom chunk unload removes mobs and must not crash navigation");
            assertEquals(NavigationState.CANCELLED, controller.state(),
                    "a controller cannot keep navigating a detached entity");
            controller.close();
        } finally {
            if (!mob.isRemoved()) mob.remove();
        }
    }

    @Test
    void customDimensionKeepsWholeFollowerInsideBothBuildLimits(Env env) {
        int minY = -16;
        int maxY = 16;
        var dimension = env.process().dimensionType().register(
                Key.key("atlas", "boundary_" + UUID.randomUUID()),
                DimensionType.builder().minY(minY).height(maxY - minY)
                        .logicalHeight(maxY - minY).build());
        Instance instance = env.process().instance()
                .createInstanceContainer(dimension);
        instance.setGenerator(unit -> {
            unit.modifier().fillHeight(minY, minY + 1, Block.STONE);
            unit.modifier().fillHeight(maxY - 3, maxY - 2, Block.STONE);
        });
        ChunkRange.chunksInRange(0, 0, 2,
                (x, z) -> instance.loadChunk(x, z).join());

        try (var service = new AsyncEntityPathfindingService(1, 8)) {
            assertBuildLimitRoute(env, instance, service, minY + 1,
                    minY, maxY);
            assertBuildLimitRoute(env, instance, service, maxY - 2,
                    minY, maxY);
        }
    }

    @Test
    void instanceTransferDuringSearchRecoversAgainstTheNewWorld(Env env) {
        PursuitHarness.SearchGate gate = new PursuitHarness.SearchGate();
        Instance first = PursuitHarness.loadedGatedFlatInstance(env, gate);
        Instance second = preparedFlat(env);
        for (int y = 40; y <= 42; y++) {
            second.setBlock(5, y, 0, Block.STONE);
        }
        EntityCreature mob = spawn(first, new Pos(0.5, 40, 0.5));

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            EntityNavigationController controller = navigation.controller(mob);
            controller.moveTo(new Pos(10.5, 40, 0.5));
            awaitActiveSearch(navigation);
            mob.setInstance(second, new Pos(0.5, 40, 0.5)).join();
            gate.land(navigation);

            runUntilTerminal(env, controller, 1_000);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    "a route landing after transfer must recover against the new instance");
            assertEquals(second, mob.getInstance());
            controller.close();
        } finally {
            gate.openPermanently();
            if (!mob.isRemoved()) mob.remove();
        }
    }

    @Test
    void destinationChunkCanUnloadAndReloadWhileSearchIsActive(Env env) {
        PursuitHarness.SearchGate gate = new PursuitHarness.SearchGate();
        Instance instance = PursuitHarness.loadedGatedFlatInstance(env, gate);
        EntityCreature mob = spawn(instance, new Pos(2.5, 40, 2.5));

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            EntityNavigationController controller = navigation.controller(mob);
            controller.moveTo(new Pos(20.5, 40, 2.5));
            awaitActiveSearch(navigation);

            instance.unloadChunk(1, 0);
            env.tick();
            assertNull(instance.getChunk(1, 0),
                    "the fixture must actually remove the destination chunk");
            instance.loadChunk(1, 0).join();
            gate.land(navigation);

            runUntilTerminal(env, controller, 900);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    "a restored destination chunk must remain searchable");
            controller.close();
        } finally {
            gate.openPermanently();
            if (!mob.isRemoved()) mob.remove();
        }
    }

    @Test
    void spatialPursuitsCloseWhenTargetLeavesTheirLifecycle(Env env) {
        Instance first = preparedFlat(env);
        Instance second = preparedFlat(env);

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8)
                .sharedMesh(SharedMeshOptions.ENABLED).build()) {
            assertPursuitClosesAfterTargetRemoval(
                    first, navigation, EntityType.BEE);
            assertPursuitClosesAfterTargetTransfer(
                    first, second, navigation, EntityType.DOLPHIN);
        }
    }

    private static EntityNavigationController controller(
            EntityCreature mob, AsyncEntityPathfindingService service) {
        return new EntityNavigationController(mob, service,
                BuiltinNavigationProfiles.forEntity(mob), SPEED);
    }

    private static EntityCreature spawn(Instance instance, Pos position) {
        EntityCreature mob = new EntityCreature(EntityType.ZOMBIE);
        mob.setInstance(instance, position).join();
        return mob;
    }

    private static Instance preparedFlat(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 2,
                (x, z) -> instance.loadChunk(x, z).join());
        return instance;
    }

    private static void assertBuildLimitRoute(
            Env env, Instance instance, AsyncEntityPathfindingService service,
            int y, int minY, int maxY) {
        EntityCreature mob = spawn(instance, new Pos(-6.5, y, 0.5));
        try {
            EntityNavigationController controller = controller(mob, service);
            controller.moveTo(new Pos(6.5, y, 0.5));
            runUntilTerminal(env, controller, 700);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    "custom-dimension boundary route did not complete at y=" + y);
            for (PathNode node : controller.nodes()) {
                double feet = node.y();
                assertFalse(feet < minY
                                || feet + mob.getBoundingBox().height() > maxY,
                        "waypoint places part of the mob outside build height: "
                                + node);
            }
            controller.close();
        } finally {
            if (!mob.isRemoved()) mob.remove();
        }
    }

    private static void assertPursuitClosesAfterTargetRemoval(
            Instance instance, NavigationSystem navigation, EntityType actorType) {
        EntityCreature actor = new EntityCreature(actorType);
        Entity target = new Entity(EntityType.ARMOR_STAND);
        target.setNoGravity(true);
        actor.setInstance(instance, new Pos(0.5, 42, 0.5)).join();
        target.setInstance(instance, new Pos(8.5, 42, 0.5)).join();
        try {
            SharedMeshPursuit pursuit = navigation.sharedMesh()
                    .pursue(actor, target, 1, 0);
            target.remove();
            pursuit.tick(1, 1);
            assertTrue(pursuit.closed(),
                    "removed target left a " + actorType.name() + " pursuit open");
            assertEquals(NavigationState.CANCELLED, pursuit.state());
        } finally {
            if (!actor.isRemoved()) actor.remove();
            if (!target.isRemoved()) target.remove();
        }
    }

    private static void assertPursuitClosesAfterTargetTransfer(
            Instance first, Instance second, NavigationSystem navigation,
            EntityType actorType) {
        EntityCreature actor = new EntityCreature(actorType);
        Entity target = new Entity(EntityType.ARMOR_STAND);
        target.setNoGravity(true);
        actor.setInstance(first, new Pos(0.5, 40, 0.5)).join();
        target.setInstance(first, new Pos(8.5, 40, 0.5)).join();
        try {
            SharedMeshPursuit pursuit = navigation.sharedMesh()
                    .pursue(actor, target, 1, 0);
            target.setInstance(second, new Pos(8.5, 40, 0.5)).join();
            pursuit.tick(1, 1);
            assertTrue(pursuit.closed(),
                    "cross-instance target left a " + actorType.name()
                            + " pursuit open");
            assertEquals(NavigationState.CANCELLED, pursuit.state());
        } finally {
            if (!actor.isRemoved()) actor.remove();
            if (!target.isRemoved()) target.remove();
        }
    }

    private static void runUntilTerminal(
            Env env, EntityNavigationController controller, int maxTicks) {
        long deadline = System.nanoTime() + Duration.ofSeconds(20).toNanos();
        for (int tick = 0; tick < maxTicks; tick++) {
            if (controller.state() == NavigationState.COMPLETED
                    || controller.state() == NavigationState.STUCK
                    || controller.state() == NavigationState.FAILED) return;
            if (System.nanoTime() > deadline) {
                fail("navigation timed out in state " + controller.state());
            }
            controller.tick();
            env.tick();
        }
    }

    private static void awaitActiveSearch(NavigationSystem navigation) {
        long deadline = System.nanoTime() + Duration.ofSeconds(5).toNanos();
        while (navigation.metricsSnapshot().searches().inFlight() == 0) {
            if (System.nanoTime() > deadline) fail("search never became active");
            Thread.onSpinWait();
        }
    }
}
