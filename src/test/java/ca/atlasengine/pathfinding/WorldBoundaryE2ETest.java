package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.event.RouteReplanEvent;
import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.result.PathStatus;
import ca.atlasengine.pathfinding.result.PathStop;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import net.minestom.server.coordinate.ChunkRange;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;
import net.minestom.testing.Env;
import net.minestom.testing.EnvTest;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

/**
 * Where navigation meets the edges of the world it is given: chunks that are
 * not loaded, other instances, and edits made off the ticking thread.
 */
@EnvTest
class WorldBoundaryE2ETest {
    @Test
    void unloadedTerrainIsAWallRatherThanAFailedSearch(Env env) throws Exception {
        Instance instance = env.createFlatInstance();
        instance.loadChunk(0, 0).join();
        EntityCreature mob = TestWorlds.spawn(instance, EntityType.ZOMBIE,
                new Pos(8.5, 40, 8.5));
        env.tick();
        Vec beyondLoaded = new Vec(40.5, 40, 8.5);

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            PathResult direct = navigation.submit(NavigationRequest.builder(
                            instance, mob.getPosition(), beyondLoaded,
                            mob.getBoundingBox(),
                            BuiltinNavigationProfiles.forEntity(mob)).build())
                    .get(10, TimeUnit.SECONDS);
            assertEquals(PathStatus.PARTIAL, direct.status(),
                    "a search reaching unloaded chunks ends short of them");
            assertEquals(PathStop.TERRAIN_UNLOADED, direct.stop(),
                    "unloaded terrain is not proof the target is unreachable");

            EntityNavigationController controller = navigation.controller(mob);
            controller.moveTo(beyondLoaded);
            for (int i = 0; i < 200; i++) {
                controller.tick();
                env.tick();
            }
            assertNotEquals(NavigationState.FAILED, controller.state());
            assertTrue(mob.getPosition().x() > 14,
                    "the mob walked to the edge of loaded terrain, at "
                            + mob.getPosition());
            assertEquals(0, navigation.metricsSnapshot().searches().failed(),
                    "no search threw");
            assertTrue(controller.targetUnreachable());
            assertEquals(PathStop.TERRAIN_UNLOADED, controller.routeStop(),
                    "the caller can tell an unloaded edge from a dead end");

            // Once the terrain loads, asking again finishes the journey.
            for (int chunkX = 1; chunkX <= 3; chunkX++) {
                instance.loadChunk(chunkX, 0).join();
            }
            controller.moveTo(beyondLoaded);
            TestWorlds.tickUntil(env, controller,
                    () -> controller.state() == NavigationState.COMPLETED,
                    () -> "never arrived once loaded: " + controller.state()
                            + " at " + mob.getPosition());
        } finally {
            mob.remove();
        }
    }

    @Test
    void anEditInAnotherInstanceLeavesRoutesAlone(Env env) {
        Instance first = env.createFlatInstance();
        Instance second = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 1, (x, z) -> {
            first.loadChunk(x, z).join();
            second.loadChunk(x, z).join();
        });
        EntityCreature mob = TestWorlds.spawn(first, EntityType.ZOMBIE,
                new Pos(0.5, 40, 0.5));
        env.tick();

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            navigation.watchBlockChanges(second);
            EntityNavigationController controller = navigation.controller(mob);
            controller.moveTo(new Vec(12.5, 40, 0.5));
            TestWorlds.awaitRoute(env, controller);
            int before = controller.generation();

            // The same coordinates as the route, in a different world.
            for (int y = 40; y < 43; y++) second.setBlock(6, y, 0, Block.STONE);
            for (int i = 0; i < 5; i++) {
                controller.tick();
                env.tick();
            }
            assertEquals(before, controller.generation(),
                    "a route is only replanned for its own instance");
            controller.close();
        } finally {
            mob.remove();
        }
    }

    @Test
    void anEditOffTheTickThreadReplansOnIt(Env env) throws Exception {
        Instance instance = TestWorlds.loadedFlat(env, 1);
        EntityCreature mob = TestWorlds.spawn(instance, EntityType.ZOMBIE,
                new Pos(0.5, 40, 0.5));
        env.tick();

        List<Thread> replanThreads = new CopyOnWriteArrayList<>();
        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            navigation.eventNode().addListener(RouteReplanEvent.class, e -> {
                if (e.reason() == RouteReplanEvent.Reason.BLOCK_CHANGE) {
                    replanThreads.add(Thread.currentThread());
                }
            });
            EntityNavigationController controller = navigation.controller(mob);
            controller.moveTo(new Vec(12.5, 40, 0.5));
            TestWorlds.awaitRoute(env, controller);

            Thread editor = Thread.ofPlatform().start(() ->
                    navigation.notifyBlockChanged(instance, new Vec(6, 40, 0)));
            editor.join();
            assertTrue(replanThreads.isEmpty(),
                    "nothing is replanned on the editing thread");

            controller.tick();
            assertEquals(List.of(Thread.currentThread()), replanThreads,
                    "the replan is announced by the tick that performs it");
            controller.close();
        } finally {
            mob.remove();
        }
    }
}
