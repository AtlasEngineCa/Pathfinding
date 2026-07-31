package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.NavigationPlan;
import net.minestom.server.coordinate.ChunkRange;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
import net.minestom.testing.Env;
import net.minestom.testing.EnvTest;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Pins the follower's refusal of a plan the entity has already walked away
 * from, in both directions across the tolerance.
 *
 * <p>The tolerance is {@code max(1.5, boundingBox.width())}, so a narrow mob
 * exercises the floor and a wide mob exercises its own width. The drift is
 * applied by moving the entity after the plan was built, which is exactly
 * what a search landing late does to a mob that kept walking.</p>
 */
@EnvTest
class PlanFollowToleranceTest {
    private static final Pos HOME = new Pos(0.5, 40, 0.5);
    /** Distance either side of the tolerance each fixture drifts. */
    private static final double MARGIN = 0.1;

    @Test
    void narrowFollowerRefusesDriftBeyondTheFixedFloor(Env env) {
        Instance instance = prepared(env);
        try (var service = new AsyncEntityPathfindingService(1, 8)) {
            EntityCreature zombie = spawn(
                    instance, EntityType.ZOMBIE, HOME);
            assertTrue(zombie.getBoundingBox().width() < 1.5,
                    "this fixture must exercise the floor, not the width");
            EntityNavigationController controller = controller(zombie, service);

            assertRefused(controller, zombie, 1.5 + MARGIN);
            assertFollowed(controller, zombie, 1.5 - MARGIN);
        }
    }

    @Test
    void wideFollowerAcceptsDriftUpToItsOwnWidth(Env env) {
        Instance instance = prepared(env);
        try (var service = new AsyncEntityPathfindingService(1, 8)) {
            EntityCreature ravager = spawn(
                    instance, EntityType.RAVAGER, HOME);
            double width = ravager.getBoundingBox().width();
            assertTrue(width - MARGIN > 1.5,
                    "this fixture must exercise the width, not the floor");
            EntityNavigationController controller = controller(ravager, service);

            assertRefused(controller, ravager, width + MARGIN);
            assertFollowed(controller, ravager, width - MARGIN);
        }
    }

    private static void assertRefused(EntityNavigationController controller,
                                      Entity entity, double drift) {
        NavigationPlan plan = driftedPlan(entity, drift);
        NavigationState before = controller.state();
        List<PathNode> route = controller.nodes();
        long stale = controller.metrics().snapshot().stalePlans();

        IllegalArgumentException refusal = assertThrows(
                IllegalArgumentException.class, () -> controller.follow(plan));

        assertEquals("entity is too far from plan start", refusal.getMessage());
        assertEquals(stale + 1, controller.metrics().snapshot().stalePlans(),
                "a refused plan must be counted exactly once");
        assertEquals(before, controller.state());
        assertEquals(route, controller.nodes());
    }

    private static void assertFollowed(EntityNavigationController controller,
                                       Entity entity, double drift) {
        NavigationPlan plan = driftedPlan(entity, drift);
        long stale = controller.metrics().snapshot().stalePlans();

        controller.follow(plan);

        assertEquals(NavigationState.FOLLOWING, controller.state());
        assertEquals(plan.nodes(), controller.nodes());
        assertEquals(stale, controller.metrics().snapshot().stalePlans(),
                "an accepted plan must not be counted stale");
    }

    /**
     * A plan computed from {@link #HOME} after which the entity has moved
     * {@code drift} blocks sideways.
     */
    private static NavigationPlan driftedPlan(Entity entity, double drift) {
        entity.teleport(HOME).join();
        Pos target = HOME.withX(HOME.x() + 6);
        NavigationPlan plan = new NavigationPlan(HOME, target,
                entity.getBoundingBox(),
                BuiltinNavigationProfiles.forEntity(entity), PathStatus.FOUND,
                List.of(node(HOME), node(target)), 2, 8);
        entity.teleport(HOME.withZ(HOME.z() + drift)).join();
        assertEquals(drift, entity.getPosition().distance(plan.start()), 1.0e-9,
                "the fixture must place the entity exactly where it intends");
        return plan;
    }

    private static PathNode node(Pos point) {
        return new PathNode(point.x(), point.y(), point.z(),
                PathNode.Movement.WALK);
    }

    private static EntityNavigationController controller(
            Entity entity, AsyncEntityPathfindingService service) {
        return new EntityNavigationController(entity, service,
                BuiltinNavigationProfiles.forEntity(entity), 0.18);
    }

    private static EntityCreature spawn(
            Instance instance, EntityType type, Pos position) {
        EntityCreature creature = new EntityCreature(type);
        creature.setInstance(instance, position).join();
        return creature;
    }

    private static Instance prepared(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 3,
                (x, z) -> instance.loadChunk(x, z).join());
        return instance;
    }
}
