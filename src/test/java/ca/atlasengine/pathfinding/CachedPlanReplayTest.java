package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.NavigationPlanCache;
import net.minestom.server.coordinate.ChunkRange;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
import net.minestom.testing.Env;
import net.minestom.testing.EnvTest;
import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Pins a controller replaying a stored plan: the cache hands back the route
 * it kept, the follower accepts it without another search, and the mob walks
 * that route to the goal.
 */
@EnvTest
class CachedPlanReplayTest {
    private static final Pos HOME = new Pos(0.5, 40, 0.5);
    private static final Pos GOAL = new Pos(6.5, 40, 0.5);
    private static final long WORLD_REVISION = 11;

    @Test
    void controllerWalksCachedPlanWithoutSubmittingAnotherSearch(Env env) {
        Instance instance = prepared(env);
        try (var service = new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature zombie = spawn(
                    instance, EntityType.ZOMBIE, HOME);
            NavigationProfile profile =
                    BuiltinNavigationProfiles.forEntity(zombie);
            EntityNavigationController controller =
                    new EntityNavigationController(
                            zombie, service, profile, 0.18);
            zombie.controller = controller;
            NavigationPlan plan = planned(instance, zombie, profile);
            NavigationPlanCache<String> cache = new NavigationPlanCache<>(4);

            cache.put("patrol:home-to-goal", WORLD_REVISION, plan);
            NavigationPlan replayed = cache.get("patrol:home-to-goal",
                            WORLD_REVISION, zombie.getPosition(), 1.0)
                    .orElseThrow();
            controller.follow(replayed);

            assertEquals(NavigationState.FOLLOWING, controller.state());
            assertEquals(plan.nodes(), controller.nodes(),
                    "the replayed route is the stored one");

            runUntilFinished(env, controller, 600);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + zombie.getPosition()
                            + ", nodeIndex=" + controller.nodeIndex()
                            + ", nodes=" + controller.nodes());
            assertTrue(zombie.getPosition().distance(GOAL) <= 0.6,
                    () -> "replay stopped short at " + zombie.getPosition());
            assertTrue(controller.nodeIndex() >= controller.nodes().size() - 1,
                    () -> "route was abandoned at node "
                            + controller.nodeIndex());
            assertEquals(0, service.metrics().snapshot().searches().submitted(),
                    "a replayed plan must reach the goal with no search at all");
            assertEquals(0, service.metrics().snapshot().stalePlans());
            double strayed = maximumDeviation(zombie.positions, plan.nodes());
            assertTrue(strayed <= 0.75,
                    () -> "mob left the cached route by " + strayed);
        }
    }

    private static NavigationPlan planned(Instance instance,
                                          EntityCreature zombie,
                                          NavigationProfile profile) {
        NavigationRequest request = NavigationRequest.builder(instance,
                        zombie.getPosition(), GOAL, zombie.getBoundingBox(),
                        profile)
                .maxPathLength(24).nodeSearchRange(24).build();
        NavigationPlan plan = NavigationPlan.from(request,
                new EntityPathfinder().findPath(request, SearchControl.NONE));
        assertEquals(PathStatus.FOUND, plan.status());
        return plan;
    }

    /** Furthest any sampled position sat from the cached polyline. */
    private static double maximumDeviation(
            List<Pos> positions, List<PathNode> nodes) {
        double worst = 0;
        for (Pos position : positions) {
            double closest = Double.POSITIVE_INFINITY;
            for (int i = 1; i < nodes.size(); i++) {
                closest = Math.min(closest, distanceToSegment(position,
                        nodes.get(i - 1).asVec(), nodes.get(i).asVec()));
            }
            worst = Math.max(worst, closest);
        }
        return worst;
    }

    private static double distanceToSegment(Point point, Point from, Point to) {
        Point span = to.sub(from);
        double length = span.x() * span.x() + span.y() * span.y()
                + span.z() * span.z();
        if (length <= 1.0e-12) return point.distance(from);
        Point offset = point.sub(from);
        double along = (offset.x() * span.x() + offset.y() * span.y()
                + offset.z() * span.z()) / length;
        double clamped = Math.clamp(along, 0, 1);
        return point.distance(from.add(span.mul(clamped)));
    }

    private static void runUntilFinished(
            Env env, EntityNavigationController controller, int maximumTicks) {
        assertTimeoutPreemptively(Duration.ofSeconds(10), () -> {
            for (int tick = 0; tick < maximumTicks; tick++) {
                env.tick();
                if (controller.state() == NavigationState.COMPLETED
                        || controller.state() == NavigationState.STUCK
                        || controller.state() == NavigationState.FAILED
                        || controller.state() == NavigationState.CANCELLED) {
                    return;
                }
                Thread.yield();
            }
        });
    }

    private static ControlledCreature spawn(
            Instance instance, EntityType type, Pos position) {
        ControlledCreature creature = new ControlledCreature(type);
        creature.setInstance(instance, position).join();
        return creature;
    }

    private static Instance prepared(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 3,
                (x, z) -> instance.loadChunk(x, z).join());
        return instance;
    }

    private static final class ControlledCreature extends EntityCreature {
        private EntityNavigationController controller;
        private final List<Pos> positions = new ArrayList<>();

        private ControlledCreature(EntityType entityType) {
            super(entityType);
        }

        @Override
        public void update(long time) {
            positions.add(getPosition());
            if (controller != null) controller.tick();
            super.update(time);
        }
    }
}
