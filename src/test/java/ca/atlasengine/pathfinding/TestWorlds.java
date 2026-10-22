package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.internal.movement.NavigationRequestFactory;
import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import net.minestom.server.coordinate.ChunkRange;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
import net.minestom.testing.Env;

import java.time.Duration;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static org.junit.jupiter.api.Assertions.fail;

/** Live-server fixtures shared by the end-to-end tests. */
public final class TestWorlds {
    private static final Duration DEFAULT_TIMEOUT = Duration.ofSeconds(10);

    private TestWorlds() {
    }

    /** A flat instance with every chunk within {@code chunkRadius} of the origin loaded. */
    public static Instance loadedFlat(Env env, int chunkRadius) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, chunkRadius,
                (x, z) -> instance.loadChunk(x, z).join());
        return instance;
    }

    public static EntityCreature spawn(Instance instance, EntityType type,
                                       Pos position) {
        EntityCreature creature = new EntityCreature(type);
        creature.setInstance(instance, position).join();
        return creature;
    }

    /**
     * Ticks the controller and then the server until {@code done} holds,
     * failing with {@code failure} after ten seconds of wall-clock time.
     */
    public static void tickUntil(Env env, EntityNavigationController controller,
                                 BooleanSupplier done, Supplier<String> failure) {
        long deadline = System.nanoTime() + DEFAULT_TIMEOUT.toNanos();
        while (!done.getAsBoolean()) {
            if (System.nanoTime() > deadline) fail(failure.get());
            controller.tick();
            env.tick();
        }
    }

    /**
     * A controller submitting straight to a test-owned service, outside the
     * admission layer a {@link NavigationSystem} puts in front of it, so a
     * test controls the worker pool and queue it plans against.
     */
    public static EntityNavigationController controller(
            Entity entity, AsyncEntityPathfindingService service,
            NavigationProfile profile, double movementPerTick) {
        return direct(entity, service, profile, false, movementPerTick);
    }

    /** As {@link #controller}, with the entity type's built-in profile. */
    public static EntityNavigationController builtinController(
            Entity entity, AsyncEntityPathfindingService service,
            double movementPerTick) {
        return direct(entity, service,
                BuiltinNavigationProfiles.forEntity(entity), true,
                movementPerTick);
    }

    private static EntityNavigationController direct(
            Entity entity, AsyncEntityPathfindingService service,
            NavigationProfile profile, boolean builtin,
            double movementPerTick) {
        return new EntityNavigationController(entity, service.metrics(),
                profile, movementPerTick, builtin,
                MovementExecutionMode.PHYSICS_VELOCITY,
                NavigationRequestFactory.entityRandomOrNull(entity),
                (key, request, priority) -> service.submitLatest(key, request),
                new ControllerSchedulingOptions(1, service.parallelism(),
                        ControllerSchedulingOptions.DEFAULT_MINIMUM_SEARCH_STALL_TICKS,
                        ControllerSchedulingOptions.DEFAULT_SHED_BACKOFF_TICKS));
    }

    /** Ticks until the controller holds a route. */
    public static void awaitRoute(Env env, EntityNavigationController controller) {
        tickUntil(env, controller, () -> !controller.nodes().isEmpty(),
                () -> "no route was produced");
    }
}
