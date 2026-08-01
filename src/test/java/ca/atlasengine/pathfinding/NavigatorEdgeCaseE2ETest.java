package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.search.EntityTraversalState;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import net.minestom.server.coordinate.ChunkRange;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;
import net.minestom.testing.Env;
import net.minestom.testing.EnvTest;
import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;

/**
 * Black-box navigator checks using Minestom's real entity tick and collision
 * pipeline. These deliberately exercise complete search-to-physics runs; a
 * node-only result cannot satisfy any assertion in this class.
 */
@EnvTest
class NavigatorEdgeCaseE2ETest {

    @Test
    void bothSpiderFamiliesClimbFromOffCentreApproaches(Env env) {
        Instance instance = preparedFlat(env);
        for (int y = 40; y <= 46; y++) {
            for (int z = -32; z <= 32; z++) {
                instance.setBlock(3, y, z, Block.STONE);
            }
        }
        for (int x = 4; x <= 8; x++) {
            for (int z = -32; z <= 32; z++) {
                instance.setBlock(x, 46, z, Block.STONE);
            }
        }

        try (var service = new AsyncEntityPathfindingService(1, 16)) {
            assertWallClimb(env, instance, service, EntityType.SPIDER, -2.25);
            assertWallClimb(env, instance, service, EntityType.CAVE_SPIDER, 2.25);
        }
    }

    @Test
    void wallClimberCanDescendAfterCrossingTheCrest(Env env) {
        Instance instance = preparedFlat(env);
        for (int y = 40; y <= 44; y++) {
            for (int z = -32; z <= 32; z++) {
                instance.setBlock(3, y, z, Block.STONE);
            }
        }

        try (var service = new AsyncEntityPathfindingService(1, 16)) {
            TrackedCreature spider = spawn(
                    instance, EntityType.SPIDER, new Pos(0.5, 40, 1.8));
            EntityNavigationController controller = controller(spider, service);
            NavigationProfile profile = BuiltinNavigationProfiles
                    .forEntityType(EntityType.SPIDER);
            Pos goal = new Pos(6.5, 40, -1.2);
            followReplay(service, instance, spider, controller, profile, goal, 24);

            runUntilTerminal(env, controller, 5_000);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + spider.getPosition()
                            + ", nodes=" + controller.nodes());
            assertTrue(spider.positions.stream().mapToDouble(Pos::y)
                            .max().orElseThrow() >= 44.7,
                    "spider never crossed the wall crest");
            assertTrue(spider.getPosition().y() < 40.2,
                    () -> "spider remained stranded above the goal: "
                            + spider.getPosition());
            assertProgressNeverFreezes(spider.positions, 160);
        }
    }

    @Test
    void groundFollowerStaysOnOneBlockShorelineLane(Env env) {
        Instance instance = preparedFlat(env);
        for (int x = -2; x <= 10; x++) {
            instance.setBlock(x, 39, -1, Block.WATER);
            instance.setBlock(x, 40, 1, Block.STONE);
            instance.setBlock(x, 41, 1, Block.STONE);
        }

        try (var service = new AsyncEntityPathfindingService(1, 16)) {
            TrackedCreature zombie = spawn(
                    instance, EntityType.ZOMBIE, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller = controller(zombie, service);
            controller.moveTo(new Pos(8.5, 40, 0.5));

            runUntilTerminal(env, controller, 1_200);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + zombie.getPosition()
                            + ", nodes=" + controller.nodes());
            double halfDepth = zombie.getBoundingBox().depth() * 0.5;
            for (Pos position : zombie.positions) {
                assertTrue(position.z() - halfDepth >= -1.0e-5,
                        () -> "zombie bounding box entered shoreline water at "
                                + position);
                assertFalse(position.z() + halfDepth > 1.0 + 1.0e-5,
                        () -> "zombie bounding box clipped shoreline wall at "
                                + position);
            }
        }
    }

    @Test
    void camelUsesItsBuiltInOneAndHalfBlockStepCapability(Env env) {
        Instance instance = preparedFlat(env);
        Block lowerSlab = Block.STONE_SLAB.withProperty("type", "bottom");
        for (int x = 3; x <= 10; x++) {
            for (int z = -4; z <= 4; z++) {
                instance.setBlock(x, 40, z, Block.STONE);
                instance.setBlock(x, 41, z, lowerSlab);
            }
        }

        try (var service = new AsyncEntityPathfindingService(1, 16)) {
            TrackedCreature camel = spawn(
                    instance, EntityType.CAMEL, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller = controller(camel, service);
            controller.moveTo(new Pos(7.5, 41.5, 0.5));

            runUntilTerminal(env, controller, 2_000);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + camel.getPosition()
                            + ", nodes=" + controller.nodes());
            assertTrue(camel.positions.stream().mapToDouble(Pos::y)
                            .max().orElseThrow() >= 41.49,
                    () -> "camel never mounted the 1.5-block surface: "
                            + camel.getPosition());
            assertProgressNeverFreezes(camel.positions, 160);
        }
    }

    private static void assertWallClimb(
            Env env, Instance instance, AsyncEntityPathfindingService service,
            EntityType type, double startZ) {
        TrackedCreature spider = spawn(
                instance, type, new Pos(0.5, 40, startZ));
        EntityNavigationController controller = controller(spider, service);
        NavigationProfile profile = BuiltinNavigationProfiles.forEntityType(type);
        Pos goal = new Pos(4.5, 47, startZ + 0.4);
        followReplay(service, instance, spider, controller, profile, goal, 24);

        runUntilTerminal(env, controller, 5_000);

        assertEquals(NavigationState.COMPLETED, controller.state(),
                () -> type + " position=" + spider.getPosition()
                        + ", nodes=" + controller.nodes());
        assertTrue(spider.getPosition().y() >= 46.7,
                () -> type + " did not reach the upper surface: "
                        + spider.getPosition());
        assertTrue(spider.positions.stream().anyMatch(position ->
                        position.x() > 2.0 && position.x() < 3.0
                                && position.y() > 41),
                () -> type + " never physically climbed the wall face");
        assertProgressNeverFreezes(spider.positions, 160);
        spider.remove();
    }

    private static void followReplay(
            AsyncEntityPathfindingService service, Instance instance,
            TrackedCreature creature, EntityNavigationController controller,
            NavigationProfile profile, Pos goal, int range) {
        NavigationRequest request = new NavigationRequest(
                instance, creature.getPosition(), goal,
                creature.getBoundingBox(), profile, range, 1, 1,
                List.of(), 63, EntityTraversalState.GROUNDED, range);
        controller.follow(AdaptiveMeshTestSupport.promoteAndReplay(
                service, request));
    }

    private static EntityNavigationController controller(
            TrackedCreature creature, AsyncEntityPathfindingService service) {
        EntityNavigationController controller = EntityNavigationController
                .builtin(creature, service, 0.16);
        creature.controller = controller;
        return controller;
    }

    private static void assertProgressNeverFreezes(
            List<Pos> positions, int maximumStationaryTicks) {
        int stationary = 0;
        int worst = 0;
        for (int i = 1; i < positions.size(); i++) {
            if (positions.get(i).distance(positions.get(i - 1)) < 1.0e-7) {
                stationary++;
                worst = Math.max(worst, stationary);
            } else {
                stationary = 0;
            }
        }
        int longestFreeze = worst;
        assertTrue(longestFreeze < maximumStationaryTicks,
                () -> "follower froze for " + longestFreeze
                        + " consecutive ticks");
    }

    private static void runUntilTerminal(
            Env env, EntityNavigationController controller, int maximumTicks) {
        assertTimeoutPreemptively(Duration.ofSeconds(12), () -> {
            for (int tick = 0; tick < maximumTicks; tick++) {
                env.tick();
                if (controller.state() == NavigationState.COMPLETED
                        || controller.state() == NavigationState.STUCK
                        || controller.state() == NavigationState.FAILED
                        || controller.state() == NavigationState.CANCELLED) {
                    return;
                }
                if (controller.state() == NavigationState.COMPUTING) {
                    Thread.sleep(1);
                } else {
                    Thread.yield();
                }
            }
        });
    }

    private static Instance preparedFlat(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 3,
                (x, z) -> instance.loadChunk(x, z).join());
        return instance;
    }

    private static TrackedCreature spawn(
            Instance instance, EntityType type, Pos position) {
        TrackedCreature creature = new TrackedCreature(type);
        creature.setInstance(instance, position).join();
        return creature;
    }

    private static final class TrackedCreature extends EntityCreature {
        private final List<Pos> positions = new ArrayList<>();
        private EntityNavigationController controller;

        private TrackedCreature(EntityType entityType) {
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
