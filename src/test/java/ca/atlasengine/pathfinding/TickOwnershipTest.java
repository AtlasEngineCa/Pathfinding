package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.adaptive.SharedMeshPursuit;
import net.minestom.server.coordinate.ChunkRange;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
import net.minestom.testing.Env;
import net.minestom.testing.EnvTest;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Method;
import java.time.Duration;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

/**
 * Who ticks what. A plain {@link EntityNavigationController} is driven by its
 * caller, while an {@link SharedMeshPursuit} drives the follower it
 * owns, and a crowd mob and a plain mob must not need the caller to know which
 * of those two an object secretly is. Ticking both freezes nothing but walks a
 * mob twice per tick; ticking neither leaves it standing still.
 */
@EnvTest
class TickOwnershipTest {
    @Test
    void pursuitHandsOutNoFollowerForACallerToTickAsWell() {
        for (Method method : SharedMeshPursuit.class.getMethods()) {
            assertFalse(EntityNavigationController.class.isAssignableFrom(
                            method.getReturnType()),
                    () -> "SharedMeshPursuit." + method.getName()
                            + "() hands out the follower the pursuit already "
                            + "ticks, so a caller cannot tell which contract "
                            + "it holds");
        }
        for (String query : new String[]{
                "state", "nodes", "generation", "spliceCounts"}) {
            assertDoesNotThrow(
                    () -> SharedMeshPursuit.class.getMethod(query),
                    () -> "a pursuit must still report " + query
                            + "() its follower was asked for");
        }
    }

    @Test
    void pursuitAdvancesOncePerTickIdHoweverManyPlacesDriveIt(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 1,
                (x, z) -> instance.loadChunk(x, z).join());
        EntityCreature actor = new EntityCreature(EntityType.ZOMBIE);
        Entity target = new Entity(EntityType.ARMOR_STAND);
        target.setNoGravity(true);
        actor.setInstance(instance, new Pos(0.5, 40, 0.5)).join();
        target.setInstance(instance, new Pos(8.5, 40, 0.5)).join();
        env.tick();

        try (NavigationSystem system = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).movementPerTick(0.2).build()) {
            SharedMeshPursuit pursuit =
                    system.sharedMesh().pursue(actor, target, 1, 0);
            // The pursuit submits on construction, and this system admits one
            // controller search at a time, so a replan issued while that first
            // search still runs is correctly held by the admission layer and
            // never reaches the counter read below. Settling first measures the
            // pursuit's decision to replan rather than the scheduler's timing.
            settle(system);
            long before = system.metricsSnapshot().searches().submitted();

            pursuit.tick(2, 1);
            long once = system.metricsSnapshot().searches().submitted() - before;
            assertTrue(once > 0, "a changed world must replan on its own tick");

            settle(system);
            pursuit.tick(3, 1);
            long twice = system.metricsSnapshot().searches().submitted() - before;
            assertEquals(once, twice,
                    "the same tick id drove the pursuit a second time");

            // The revision the swallowed call carried is not lost: the next
            // genuine tick still sees the world it announced.
            pursuit.tick(3, 2);
            assertTrue(system.metricsSnapshot().searches().submitted() - before
                            > twice,
                    "a revision announced twice in one tick was dropped");
            pursuit.close();
        } finally {
            actor.remove();
            target.remove();
        }
    }

    /**
     * Waits until no search is in flight, so the next submission is dispatched
     * rather than deferred by the bounded admission layer.
     */
    private static void settle(NavigationSystem system) {
        long deadline = System.nanoTime() + Duration.ofSeconds(10).toNanos();
        while (system.metricsSnapshot().searches().inFlight() > 0) {
            if (System.nanoTime() > deadline) {
                fail("searches did not settle: "
                        + system.metricsSnapshot().searches());
            }
            Thread.onSpinWait();
        }
    }
}
