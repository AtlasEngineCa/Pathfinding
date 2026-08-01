package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationModifiers;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.EntityTraversalState;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.OpenableBlockFamily;
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
import java.util.function.Consumer;

import static org.junit.jupiter.api.Assertions.*;

@EnvTest
class MinestomNavigationE2ETest {

    @Test
    void jumpEnabledMobCrossesPlatformGapWithOneLaunch(Env env) {
        Instance instance = preparedFlat(env);
        for (int y = 0; y <= 39; y++) {
            instance.setBlock(2, y, 0, Block.AIR);
        }
        for (int x = -2; x <= 6; x++) {
            for (int y = 40; y <= 43; y++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
        }

        NavigationProfile jumping = jumpProfile(2);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(
                    instance, EntityType.ZOMBIE,
                    new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(
                            mob, service, jumping, 0.18);
            mob.controller = controller;
            Pos goal = new Pos(4.5, 40, 0.5);
            NavigationRequest meshRequest = new NavigationRequest(
                    instance, mob.getPosition(), goal, mob.getBoundingBox(),
                    jumping, 16, 1, 1, List.of(), 63,
                    EntityTraversalState.GROUNDED, 16);
            controller.follow(AdaptiveMeshTestSupport.promoteAndReplay(
                    service, meshRequest));

            runUntilFinished(env, controller, 800);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition()
                            + ", nodes=" + controller.nodes());
            assertEquals(1, controller.nodes().stream().filter(node ->
                    node.movement() == PathNode.Movement.JUMP).count());
            assertEquals(1, mob.upwardLaunches,
                    "one atomic jump edge must inject vertical velocity once");
            assertTrue(mob.positions.stream().mapToDouble(Pos::y)
                    .max().orElseThrow() > 40.5);
            assertTrue(mob.positions.stream().mapToDouble(Pos::y)
                            .min().orElseThrow() >= 39.99,
                    "entity fell into the platform gap");
            emit("platform-jump-distance-2", mob, controller);
        }
    }

    @Test
    void configuredFourBlockJumpCrossesThreeUnsupportedCells(Env env) {
        Instance instance = preparedFlat(env);
        for (int x = 1; x <= 3; x++) {
            for (int y = 0; y <= 39; y++) {
                instance.setBlock(x, y, 0, Block.AIR);
            }
        }
        // Prevent a ground detour: the only route is the configured jump.
        for (int x = -2; x <= 7; x++) {
            for (int y = 40; y <= 43; y++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
        }

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(
                    instance, EntityType.ZOMBIE,
                    new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(
                            mob, service, jumpProfile(4), 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(6.5, 40, 0.5));

            runUntilFinished(env, controller, 1_000);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition()
                            + ", nodes=" + controller.nodes());
            List<PathNode> jumps = controller.nodes().stream().filter(node ->
                    node.movement() == PathNode.Movement.JUMP).toList();
            assertEquals(1, jumps.size(), controller.nodes()::toString);
            assertEquals(4, jumps.getFirst().graphX(),
                    "jump distance is measured between graph anchors");
            assertEquals(1, mob.upwardLaunches,
                    "a long platform jump must still launch exactly once");
            assertTrue(mob.positions.stream().mapToDouble(Pos::y)
                            .min().orElseThrow() >= 39.99,
                    () -> "long jump fell into the void: " + mob.positions);
            assertTrue(mob.positions.stream().mapToDouble(Pos::y)
                    .max().orElseThrow() > 40.5);
            emit("platform-jump-distance-4", mob, controller);
        }
    }

    @Test
    void liveNavigationDetoursAroundDeepGroundPitWithoutFalling(Env env) {
        Instance instance = preparedFlat(env);
        for (int x = 8; x <= 13; x++) {
            for (int z = -4; z <= 4; z++) {
                for (int y = 0; y <= 39; y++) {
                    instance.setBlock(x, y, z, Block.AIR);
                }
            }
        }

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(
                    instance, EntityType.ZOMBIE,
                    new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.24);
            mob.controller = controller;
            controller.moveTo(new Pos(24.5, 40, 0.5));

            runUntilFinished(env, controller, 1_500);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition()
                            + ", nodes=" + controller.nodes());
            assertTrue(controller.nodes().stream().anyMatch(node ->
                            Math.abs(node.graphZ()) >= 5),
                    () -> "path did not route around pit: "
                            + controller.nodes());
            assertTrue(controller.nodes().stream().noneMatch(node ->
                            node.graphX() >= 8 && node.graphX() <= 13
                                    && node.graphZ() >= -4
                                    && node.graphZ() <= 4),
                    () -> "path contains an unsupported pit node: "
                            + controller.nodes());
            assertTrue(mob.positions.stream().mapToDouble(Pos::y)
                            .min().orElseThrow() >= 39.99,
                    () -> "entity fell while routing around pit: "
                            + mob.positions);
            assertTrajectoryRemainsSupported(mob);
            assertTrue(traveled(mob.positions)
                            <= graphLength(controller.nodes()) + 0.75,
                    () -> "pit follower wandered: traveled="
                            + traveled(mob.positions) + ", graph="
                            + graphLength(controller.nodes()));
            emit("deep-pit-detour", mob, controller);
        }
    }

    @Test
    void partialJumpPathLandsAndStopsOnReachableIsland(Env env)
            throws InterruptedException {
        Instance instance = preparedFlat(env);
        for (int x : new int[]{1, 3, 4, 5}) {
            for (int z = -20; z <= 20; z++) {
                for (int y = 35; y <= 39; y++) {
                    instance.setBlock(x, y, z, Block.AIR);
                }
            }
        }
        for (int x = -2; x <= 8; x++) {
            for (int y = 40; y <= 43; y++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
        }

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(
                    instance, EntityType.ZOMBIE,
                    new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(
                            mob, service, jumpProfile(2), 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(6.5, 40, 0.5));

            Pos previous = mob.getPosition();
            int stationary = 0;
            for (int tick = 0; tick < 800 && stationary < 12; tick++) {
                env.tick();
                if (controller.state() == NavigationState.COMPUTING) {
                    Thread.sleep(1);
                }
                Pos current = mob.getPosition();
                if (controller.state() == NavigationState.PARTIAL
                        && current.distance(previous) < 1.0e-8) {
                    stationary++;
                } else {
                    stationary = 0;
                }
                previous = current;
            }

            assertEquals(NavigationState.PARTIAL, controller.state(),
                    () -> "position=" + mob.getPosition()
                            + ", nodes=" + controller.nodes());
            assertTrue(stationary >= 12);
            assertEquals(2, controller.nodes().getLast().graphX());
            assertEquals(PathNode.Movement.JUMP,
                    controller.nodes().getLast().movement());
            assertEquals(1, mob.upwardLaunches);
            assertTrue(mob.getPosition().x() > 1.8
                            && mob.getPosition().x() < 2.8,
                    () -> "partial endpoint missed island: "
                            + mob.getPosition());
            assertTrue(mob.getPosition().y() >= 39.99,
                    () -> "partial path fell into void: "
                            + mob.getPosition());
            emit("partial-platform-jump", mob, controller);
        }
    }

    @Test
    void changedJumpArcIsRevalidatedBeforeLaunch(Env env)
            throws InterruptedException {
        Instance instance = preparedFlat(env);
        for (int y = 0; y <= 39; y++) {
            instance.setBlock(2, y, 0, Block.AIR);
        }
        for (int x = -2; x <= 6; x++) {
            for (int y = 40; y <= 43; y++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
        }

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(
                    instance, EntityType.ZOMBIE,
                    new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(
                            mob, service, jumpProfile(2), 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(4.5, 40, 0.5));

            assertTimeoutPreemptively(Duration.ofSeconds(5), () -> {
                while (service.activeCount() != 0
                        || service.queuedCount() != 0) {
                    Thread.sleep(1);
                }
            });
            // The accepted worker result contains the jump. This new block
            // intersects its arc and must be observed by tick-thread
            // preflight validation before any launch velocity is applied.
            instance.setBlock(2, 42, 0, Block.STONE);

            for (int tick = 0; tick < 1_200
                    && controller.state() != NavigationState.COMPLETED
                    && controller.state() != NavigationState.PARTIAL
                    && controller.state() != NavigationState.FAILED; tick++) {
                env.tick();
                if (controller.state() == NavigationState.COMPUTING
                        || service.activeCount() != 0
                        || service.queuedCount() != 0) {
                    Thread.sleep(1);
                }
            }

            assertEquals(0, mob.upwardLaunches,
                    "stale jump trajectory launched through a new obstacle");
            assertTrue(controller.state() == NavigationState.COMPLETED
                            || controller.state() == NavigationState.PARTIAL
                            || controller.state() == NavigationState.FAILED,
                    () -> "blocked arc was not replanned: state="
                            + controller.state() + ", nodes="
                            + controller.nodes());
            assertTrue(controller.nodes().stream().noneMatch(node ->
                            node.movement() == PathNode.Movement.JUMP),
                    () -> "replacement path retained blocked jump: "
                            + controller.nodes());
            Pos inserted = new Pos(2, 42, 0);
            for (Pos position : mob.positions) {
                assertFalse(Block.STONE.collisionShape()
                                .intersectBox(position.sub(inserted),
                                        mob.getBoundingBox()),
                        () -> "replanned trajectory clipped inserted block at "
                                + position);
            }
        }
    }

    @Test
    void hardAlternatingMazeWorksForMultipleBoundingBoxes(Env env) {
        Instance instance = preparedFlat(env);
        List<Pos> wallBlocks = new ArrayList<>();
        addWallWithGap(instance, wallBlocks, 3, -15, 15, 3, 5);
        addWallWithGap(instance, wallBlocks, 6, -15, 15, -5, -3);
        addWallWithGap(instance, wallBlocks, 9, -15, 15, 3, 5);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            for (double width : new double[]{0.6, 1.4}) {
                ControlledCreature mob = spawn(
                        instance, EntityType.ZOMBIE,
                        new Pos(0.5, 40, 0.5));
                mob.setBoundingBox(width, 1.8, width);
                EntityNavigationController controller =
                        EntityNavigationController.builtin(
                                mob, service, 0.16);
                mob.controller = controller;
                controller.moveTo(new Pos(12.5, 40, 0.5));

                runUntilFinished(env, controller, 2_500);

                assertEquals(NavigationState.COMPLETED, controller.state(),
                        () -> "width=" + width + ", position="
                                + mob.getPosition() + ", nodes="
                                + controller.nodes() + ", nodeIndex="
                                + controller.nodeIndex() + ", tail="
                                + mob.positions.subList(
                                Math.max(0, mob.positions.size() - 20),
                                mob.positions.size()));
                assertGapUsed(controller.nodes(), 3, 3, 5);
                assertGapUsed(controller.nodes(), 6, -5, -3);
                assertGapUsed(controller.nodes(), 9, 3, 5);
                assertTrajectoryAvoidsBlocks(mob, wallBlocks);
                assertTrue(traveled(mob.positions)
                                <= graphLength(controller.nodes()) + 1.25,
                        () -> "width=" + width
                                + " follower wandered excessively: traveled="
                                + traveled(mob.positions) + ", graph="
                                + graphLength(controller.nodes()));
                emit("alternating-maze-" + width, mob, controller);
                mob.remove();
            }
        }
    }

    @Test
    void largeAlternatingTerrainCourseCompletesEfficientlyWithoutFalls(
            Env env) {
        Instance instance = preparedFlat(env);
        List<Pos> wallBlocks = new ArrayList<>();
        addWallWithGap(instance, wallBlocks, 12, -24, 24, 10, 14);
        addWallWithGap(instance, wallBlocks, 24, -24, 24, -14, -10);
        addWallWithGap(instance, wallBlocks, 36, -24, 24, 10, 14);
        addWallWithGap(instance, wallBlocks, 48, -24, 24, -14, -10);

        // Deep holes make tempting straight-line cells genuinely unsafe.
        for (int x : new int[]{6, 18, 30, 42, 54}) {
            for (int z = -2; z <= 2; z++) {
                for (int y = 0; y <= 39; y++) {
                    instance.setBlock(x, y, z, Block.AIR);
                }
            }
        }

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(
                    instance, EntityType.ZOMBIE,
                    new Pos(0.5, 40, 0.5));
            mob.setBoundingBox(1.8, 1.8, 0.8);
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.32);
            mob.controller = controller;
            controller.moveTo(new Pos(60, 40, 0.5));

            runUntilFinished(env, controller, 5_000);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition()
                            + ", nodeIndex=" + controller.nodeIndex()
                            + ", nodes=" + controller.nodes());
            assertTrajectoryUsedGap(mob, 12, 10, 14);
            assertTrajectoryUsedGap(mob, 24, -14, -10);
            assertTrajectoryUsedGap(mob, 36, 10, 14);
            assertTrajectoryUsedGap(mob, 48, -14, -10);
            assertTrajectoryAvoidsBlocks(mob, wallBlocks);
            assertTrajectoryRemainsSupported(mob);
            assertTrue(mob.positions.stream().mapToDouble(Pos::y)
                            .min().orElseThrow() >= 39.99,
                    () -> "large terrain route fell: " + mob.getPosition());
            assertTrue(traveled(mob.positions) < 250,
                    () -> "large terrain follower wandered excessively: "
                            + traveled(mob.positions));
            assertTrue(mob.positions.size() < 1_500,
                    () -> "large terrain took excessive ticks: "
                            + mob.positions.size());
            emit("large-alternating-terrain", mob, controller);
        }
    }

    @Test
    void largeMobCrossesUnevenTerracesWithoutSlidingIntoSideSlots(Env env) {
        Instance instance = preparedFlat(env);
        // A three-block-wide route rises two one-block terraces, descends
        // them again, and is bordered by drops deeper than the mob may fall.
        for (int x = -2; x <= 26; x++) {
            int top = x >= 6 && x <= 19 ? 40 : 39;
            if (x >= 10 && x <= 15) top = 41;
            for (int z = -1; z <= 1; z++) {
                for (int y = 40; y <= top; y++) {
                    instance.setBlock(x, y, z, Block.STONE);
                }
            }
            for (int z : new int[]{-2, 2}) {
                for (int y = 0; y <= 39; y++) {
                    instance.setBlock(x, y, z, Block.AIR);
                }
            }
        }

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(instance, EntityType.RAVAGER,
                    new Pos(0.5, 40, 0.5));
            mob.setBoundingBox(1.8, 2.7, 1.8);
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.22);
            mob.controller = controller;
            controller.moveTo(new Pos(24.5, 40, 0.5));

            runUntilFinished(env, controller, 2_000);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition()
                            + ", nodes=" + controller.nodes());
            assertTrue(mob.positions.stream().mapToDouble(Pos::y)
                            .max().orElseThrow() >= 42 - 1.0e-3,
                    "large mob never climbed the upper terrace");
            assertTrue(mob.positions.stream().mapToDouble(Pos::y)
                            .min().orElseThrow() >= 39.99,
                    () -> "large mob fell into a side slot: "
                            + mob.getPosition());
            assertTrue(mob.positions.stream().allMatch(position ->
                            Math.abs(position.z() - 0.5) <= 0.61),
                    () -> "large mob drifted toward an unsafe slot: "
                            + mob.positions);
            // Physics briefly lifts the leading edge slightly above a full
            // block while resolving a step. This still rejects any deep slot.
            assertTrajectoryCornersHaveNearbySupport(mob, 1.25);
            assertTrue(controller.nodes().stream().noneMatch(node ->
                            node.movement() == PathNode.Movement.JUMP),
                    "ordinary terraces must use steps, not platform jumps");
            emit("large-uneven-terraces", mob, controller);
        }
    }

    @Test
    void shapedTerrainAndHazardsWorkInLiveSimulation(Env env) {
        Instance instance = preparedFlat(env);
        instance.setBlock(2, 40, 0, Block.GRAY_CARPET);
        instance.setBlock(3, 40, 0,
                Block.STONE_SLAB.withProperty("type", "bottom"));
        instance.setBlock(4, 40, 0,
                Block.STONE_STAIRS.withProperty("facing", "east")
                        .withProperty("half", "bottom"));
        instance.setBlock(6, 40, 0, Block.CANDLE);
        instance.setBlock(7, 40, 0, Block.CACTUS);
        instance.setBlock(8, 40, 0, Block.OAK_FENCE);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(
                    instance, EntityType.ZOMBIE, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(
                            mob, service, jumpProfile(4), 0.12);
            mob.controller = controller;
            controller.moveTo(new Pos(10.5, 40, 0.5));

            runUntilFinished(env, controller, 1_000);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition()
                            + ", path=" + controller.nodes());
            assertTrue(controller.nodes().stream()
                    .anyMatch(node -> node.graphZ() != 0),
                    "cactus and fence should force a detour");
            assertTrue(controller.nodes().stream().noneMatch(node ->
                            node.movement() == PathNode.Movement.JUMP),
                    () -> "partial shapes became platform jumps: "
                            + controller.nodes());
            long stepNodes = controller.nodes().stream().filter(node ->
                    node.movement() == PathNode.Movement.STEP_UP).count();
            assertTrue(mob.upwardLaunches <= stepNodes,
                    () -> "repeated jump impulses on partial shapes: launches="
                            + mob.upwardLaunches + ", steps=" + stepNodes);
            assertTrue(mob.positions.stream()
                    .mapToDouble(Pos::y).max().orElseThrow() > 40.35,
                    "slab/stair course produced no physical elevation");
            for (Pos position : mob.positions) {
                for (int x : new int[]{7, 8}) {
                    Block obstacle = x == 7 ? Block.CACTUS : Block.OAK_FENCE;
                    Pos block = new Pos(x, 40, 0);
                    assertFalse(obstacle.collisionShape()
                                    .intersectBox(position.sub(block),
                                            mob.getBoundingBox()),
                            () -> "trajectory touched hazard/obstacle "
                                    + block + " at " + position);
                }
            }
            emit("shapes-hazards", mob, controller);
        }
    }

    @Test
    void ordinaryStairFlightUsesStepPhysicsWithoutJumpImpulses(Env env) {
        Instance instance = preparedFlat(env);
        Block stair = Block.STONE_STAIRS
                .withProperty("facing", "east")
                .withProperty("half", "bottom")
                .withProperty("shape", "straight");

        // Four ascending stairs lead to an elevated landing. Solid columns
        // beneath them prevent the fixture from accidentally measuring gaps.
        for (int step = 0; step < 4; step++) {
            int x = 2 + step;
            for (int y = 40; y < 40 + step; y++) {
                instance.setBlock(x, y, 0, Block.STONE);
            }
            instance.setBlock(x, 40 + step, 0, stair);
        }
        for (int x = 6; x <= 10; x++) {
            for (int y = 40; y <= 43; y++) {
                instance.setBlock(x, y, 0, Block.STONE);
            }
        }
        // Keep the mob on the staircase so a ground-level detour cannot make
        // the test pass without physically climbing each shaped block.
        for (int x = -1; x <= 10; x++) {
            for (int y = 40; y <= 48; y++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
        }

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(
                    instance, EntityType.ZOMBIE, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.12);
            mob.controller = controller;
            controller.moveTo(new Pos(9.5, 44, 0.5));

            runUntilFinished(env, controller, 1_200);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition()
                            + ", nodes=" + controller.nodes());
            assertTrue(controller.nodes().stream().anyMatch(node ->
                            node.movement() == PathNode.Movement.STEP_UP),
                    () -> "stair flight emitted no step-up nodes: "
                            + controller.nodes());
            assertTrue(controller.nodes().stream().noneMatch(node ->
                            node.movement() == PathNode.Movement.JUMP),
                    () -> "ordinary stairs became jump edges: "
                            + controller.nodes());
            assertEquals(0, mob.upwardLaunches,
                    "walking up ordinary stairs must not inject jump velocity");
            assertTrue(mob.positions.stream().mapToDouble(Pos::y)
                            .max().orElseThrow() >= 43.9,
                    () -> "mob never reached the elevated landing: "
                            + mob.positions);
            emit("ordinary-stair-flight", mob, controller);
        }
    }

    @Test
    void customBoundingBoxesRespectExactOneByTwoOpening(Env env) {
        Instance instance = preparedFlat(env);
        for (int z = -5; z <= 5; z++) {
            for (int y = 40; y <= 42; y++) {
                if (z == 0 && y < 42) continue;
                instance.setBlock(3, y, z, Block.STONE);
            }
        }

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature narrow = spawn(
                    instance, EntityType.ZOMBIE, new Pos(0.5, 40, 0.5));
            narrow.setBoundingBox(0.8, 1.8, 0.8);
            EntityNavigationController narrowController =
                    EntityNavigationController.builtin(
                            narrow, service, 0.14);
            narrow.controller = narrowController;
            narrowController.moveTo(new Pos(6.5, 40, 0.5));
            runUntilFinished(env, narrowController, 700);

            assertEquals(NavigationState.COMPLETED, narrowController.state());
            assertTrue(narrowController.nodes().stream()
                    .allMatch(node -> node.graphZ() == 0),
                    narrowController.nodes()::toString);
            assertTrajectoryAvoidsOpeningWall(narrow);
            emit("custom-box-0.8-gap", narrow, narrowController);
            narrow.remove();

            ControlledCreature wide = spawn(
                    instance, EntityType.ZOMBIE, new Pos(0.5, 40, 0.5));
            wide.setBoundingBox(1.01, 1.8, 1.01);
            EntityNavigationController wideController =
                    EntityNavigationController.builtin(wide, service, 0.14);
            wide.controller = wideController;
            wideController.moveTo(new Pos(6.5, 40, 0.5));
            runUntilFinished(env, wideController, 1_000);

            assertEquals(NavigationState.COMPLETED, wideController.state(),
                    () -> "position=" + wide.getPosition()
                            + ", nodes=" + wideController.nodes());
            assertTrue(wideController.nodes().stream().anyMatch(
                    node -> Math.abs(node.graphZ()) >= 6),
                    () -> "wide entity incorrectly used the opening: "
                            + wideController.nodes());
            assertTrajectoryAvoidsOpeningWall(wide);
            emit("custom-box-1.01-detour", wide, wideController);
        }
    }

    @Test
    void relevantBlockChangeRecomputesRouteAroundNewWall(Env env)
            throws InterruptedException {
        Instance instance = preparedFlat(env);
        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(
                    instance, EntityType.ZOMBIE, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.14);
            mob.controller = controller;
            controller.moveTo(new Pos(8.5, 40, 0.5));

            for (int tick = 0;
                 tick < 200 && controller.state() == NavigationState.COMPUTING;
                 tick++) {
                env.tick();
                Thread.sleep(1);
            }
            for (int y = 40; y <= 41; y++) {
                for (int z = -1; z <= 1; z++) {
                    instance.setBlock(3, y, z, Block.STONE);
                }
            }
            controller.onBlockChanged(new Pos(3, 40, 0));
            assertEquals(NavigationState.FOLLOWING, controller.state(),
                    "an asynchronous recomputation should retain and follow "
                            + "the accepted path until its replacement arrives");

            runUntilFinished(env, controller, 800);

            assertEquals(NavigationState.COMPLETED, controller.state());
            assertTrue(controller.nodes().stream().anyMatch(
                    node -> Math.abs(node.graphZ()) >= 2),
                    () -> "route was not rebuilt around the new wall: "
                            + controller.nodes());
            for (Pos position : mob.positions) {
                for (int y = 40; y <= 41; y++) {
                    for (int z = -1; z <= 1; z++) {
                        Pos block = new Pos(3, y, z);
                        assertFalse(Block.STONE.collisionShape()
                                        .intersectBox(position.sub(block),
                                                mob.getBoundingBox()),
                                () -> "recomputed trajectory clipped " + block
                                        + " at " + position);
                    }
                }
            }
        }
    }

    @Test
    void exhaustedPartialPathStopsWithoutSelfResubmission(Env env)
            throws InterruptedException {
        Instance instance = preparedFlat(env);
        for (int y = 40; y <= 42; y++) {
            for (int coordinate = -1; coordinate <= 1; coordinate++) {
                instance.setBlock(5, y, coordinate, Block.STONE);
                instance.setBlock(7, y, coordinate, Block.STONE);
                instance.setBlock(6 + coordinate, y, -1, Block.STONE);
                instance.setBlock(6 + coordinate, y, 1, Block.STONE);
            }
        }

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(
                    instance, EntityType.ZOMBIE, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(6.5, 40, 0.5));

            Pos previous = mob.getPosition();
            int stationaryTicks = 0;
            for (int tick = 0; tick < 800 && stationaryTicks < 12; tick++) {
                env.tick();
                if (controller.state() == NavigationState.COMPUTING) {
                    Thread.sleep(1);
                }
                Pos current = mob.getPosition();
                if (controller.state() == NavigationState.PARTIAL
                        && current.distance(previous) < 1.0e-8) {
                    stationaryTicks++;
                } else {
                    stationaryTicks = 0;
                }
                previous = current;
            }

            assertEquals(NavigationState.PARTIAL, controller.state());
            assertTrue(stationaryTicks >= 12,
                    "partial route never became terminal");
            Pos endpoint = mob.getPosition();
            for (int tick = 0; tick < 80; tick++) {
                env.tick();
                assertNotEquals(NavigationState.COMPUTING, controller.state(),
                        "navigation must not autonomously submit another search");
            }
            assertEquals(NavigationState.PARTIAL, controller.state());
            assertTrue(mob.getPosition().distance(endpoint) < 1.0e-6);
        }
    }

    @Test
    void entityActuallyWalksAroundWallToGoal(Env env) {
        Instance instance = preparedFlat(env);
        for (int x = -5; x <= 5; x++) {
            instance.setBlock(x, 40, 5, Block.STONE);
            instance.setBlock(x, 41, 5, Block.STONE);
        }

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(2, 32)) {
            ControlledCreature mob = spawn(instance, EntityType.ZOMBIE, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(0.5, 40, 10.5));

            runUntilFinished(env, controller, 1_200);

            assertEquals(NavigationState.COMPLETED, controller.state());
            assertTrue(mob.getPosition().distance(
                            new Pos(0.5, 40, 10.5)) <= 0.5,
                    "coordinate navigation must reach the requested goal cell");
            assertTrue(controller.nodes().stream().anyMatch(n -> Math.abs(n.x()) >= 5.5));
            assertEquals(8 * Math.sqrt(2) + 6,
                    graphLength(controller.nodes()), 1.0e-9,
                    "shortest route must take cardinal edges around the wall end; "
                            + "a diagonal there would cut the blocked corner");
            double minimumX = mob.positions.stream()
                    .mapToDouble(Pos::x).min().orElseThrow();
            double maximumX = mob.positions.stream()
                    .mapToDouble(Pos::x).max().orElseThrow();
            assertTrue(minimumX <= -5.3 || maximumX >= 6.3,
                    () -> "continuous follower never achieved entity-width wall "
                            + "clearance: x=[" + minimumX + "," + maximumX + "]");
            assertTrue(traveled(mob.positions)
                            <= graphLength(controller.nodes()) + 0.5,
                    "physics follower added excessive motion beyond the "
                            + "discrete route");
            assertTrajectoryDoesNotIntersectWall(mob);
            emit("zombie-wall", mob, controller);
        }
    }

    @Test
    void villagerOpensAndWalksThroughWoodDoor(Env env) {
        Instance instance = preparedFlat(env);
        for (int x = -3; x <= 7; x++) {
            for (int y = 40; y <= 42; y++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
        }
        Block door = Block.OAK_DOOR.withProperty("facing", "east").withProperty("hinge", "left");
        instance.setBlock(2, 40, 0, door.withProperty("half", "lower"));
        instance.setBlock(2, 41, 0, door.withProperty("half", "upper"));

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(instance, EntityType.VILLAGER, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.16);
            mob.controller = controller;
            controller.moveTo(new Pos(5.5, 40, 0.5));

            runUntilFinished(env, controller, 600);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition() + ", door="
                            + instance.getBlock(2, 40, 0).properties()
                            + ", nodes=" + controller.nodes());
            assertEquals("true", instance.getBlock(2, 40, 0).properties().get("open"));
            assertTrue(controller.nodes().stream().allMatch(node -> node.graphZ() == 0));
            assertWaypointWasReachedBeforeAdvance(mob, controller, 2);
            emit("villager-door", mob, controller);
        }
    }

    /**
     * Every manipulable family, in the orientations that matter, swept per
     * tick against the state the block actually held on that tick. An open
     * door or trapdoor is a 0.1875 slab rather than empty space and the shape
     * changes mid-run, so the final state proves nothing about the ticks the
     * entity spent crossing the cell.
     */
    @Test
    void openableBlocksAreOpenBeforeTheMobsBoxReachesThem(Env env) {
        for (String facing : new String[]{"east", "west"}) {
            for (String hinge : new String[]{"left", "right"}) {
                Block door = Block.OAK_DOOR.withProperty("facing", facing)
                        .withProperty("hinge", hinge)
                        .withProperty("open", "false");
                walkThroughOpenable(env,
                        "door facing=" + facing + " hinge=" + hinge,
                        BlockManipulationCapabilities.STANDARD,
                        List.of(new Pos(2, 40, 0), new Pos(2, 41, 0)),
                        instance -> {
                            instance.setBlock(2, 40, 0,
                                    door.withProperty("half", "lower"));
                            instance.setBlock(2, 41, 0,
                                    door.withProperty("half", "upper"));
                        });
            }
        }
        for (String facing : new String[]{"north", "south"}) {
            walkThroughOpenable(env, "trapdoor facing=" + facing,
                    BlockManipulationCapabilities.of(
                            OpenableBlockFamily.TRAPDOOR),
                    List.of(new Pos(2, 40, 0)),
                    instance -> instance.setBlock(2, 40, 0,
                            Block.OAK_TRAPDOOR.withProperty("facing", facing)
                                    .withProperty("half", "bottom")
                                    .withProperty("open", "false")));
            walkThroughOpenable(env, "gate facing=" + facing,
                    BlockManipulationCapabilities.of(
                            OpenableBlockFamily.FENCE_GATE),
                    List.of(new Pos(2, 40, 0)),
                    instance -> instance.setBlock(2, 40, 0,
                            Block.OAK_FENCE_GATE.withProperty("facing", facing)
                                    .withProperty("open", "false")));
        }
    }

    /**
     * The other half of the cycle: a block the follower shuts again must
     * never shut onto the box. Nothing pushes an entity out of a shape that
     * appears around it, so this is the one direction physics cannot correct.
     */
    @Test
    void closingBehindNeverShutsAnOpenableOnTheMobsBox(Env env) {
        Block door = Block.OAK_DOOR.withProperty("facing", "east")
                .withProperty("hinge", "left").withProperty("open", "false");
        Instance instance = walkThroughOpenable(env, "door closing behind",
                BlockManipulationCapabilities.STANDARD.closingBehind(),
                List.of(new Pos(2, 40, 0), new Pos(2, 41, 0)),
                world -> {
                    world.setBlock(2, 40, 0, door.withProperty("half", "lower"));
                    world.setBlock(2, 41, 0, door.withProperty("half", "upper"));
                });

        assertEquals("false", instance.getBlock(2, 40, 0).getProperty("open"),
                "the sweep only means something once the door has shut again");
        assertEquals("false", instance.getBlock(2, 41, 0).getProperty("open"));

        walkThroughOpenable(env, "trapdoor closing behind",
                BlockManipulationCapabilities.of(OpenableBlockFamily.TRAPDOOR)
                        .closingBehind(),
                List.of(new Pos(2, 40, 0)),
                world -> world.setBlock(2, 40, 0, Block.OAK_TRAPDOOR
                        .withProperty("facing", "north")
                        .withProperty("half", "bottom")
                        .withProperty("open", "false")));
        walkThroughOpenable(env, "gate closing behind",
                BlockManipulationCapabilities.of(OpenableBlockFamily.FENCE_GATE)
                        .closingBehind(),
                List.of(new Pos(2, 40, 0)),
                world -> world.setBlock(2, 40, 0, Block.OAK_FENCE_GATE
                        .withProperty("facing", "north")
                        .withProperty("open", "false")));
    }

    private static Instance walkThroughOpenable(
            Env env, String label, BlockManipulationCapabilities capabilities,
            List<Pos> watched, Consumer<Instance> setup) {
        Instance instance = preparedFlat(env);
        for (int x = -3; x <= 8; x++) {
            for (int y = 40; y <= 42; y++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
        }
        setup.accept(instance);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(
                    instance, EntityType.VILLAGER, new Pos(0.5, 40, 0.5));
            mob.watchedBlocks = watched;
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.16);
            mob.controller = controller;
            controller.moveTo(new Pos(6.5, 40, 0.5),
                    NavigationModifiers.builder()
                            .blockManipulation(capabilities).build());

            runUntilFinished(env, controller, 600);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> label + ": position=" + mob.getPosition()
                            + ", nodes=" + controller.nodes());
            assertNeverOverlappedAClosedOpenable(mob, label);
            mob.remove();
        }
        return instance;
    }

    @Test
    void fireNeighborWaypointCannotBeDirectionallySkipped(Env env) {
        Instance instance = preparedFlat(env);
        for (int x = -20; x <= 20; x++) {
            for (int y = 40; y <= 42; y++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
        }
        instance.setBlock(2, 40, 1, Block.MAGMA_BLOCK);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(
                    instance, EntityType.ZOMBIE, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(5.5, 40, 0.5));

            runUntilFinished(env, controller, 600);

            assertEquals(NavigationState.COMPLETED, controller.state());
            assertWaypointWasReachedBeforeAdvance(mob, controller, 2);
            assertTrue(controller.nodes().stream().allMatch(node -> node.graphZ() == 0));
            emit("zombie-fire-neighbor", mob, controller);
        }
    }

    @Test
    void temporaryDoorCapabilityPlansOpensAndTraversesDoor(Env env) {
        Instance instance = preparedFlat(env);
        for (int x = -20; x <= 20; x++) {
            for (int y = 40; y <= 42; y++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
        }
        Block door = Block.OAK_DOOR.withProperty("facing", "east")
                .withProperty("hinge", "left").withProperty("open", "false");
        instance.setBlock(2, 40, 0, door.withProperty("half", "lower"));
        instance.setBlock(2, 41, 0, door.withProperty("half", "upper"));

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature zombie = spawn(
                    instance, EntityType.ZOMBIE, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(zombie, service, 0.18);
            zombie.controller = controller;
            controller.moveTo(new Pos(5.5, 40, 0.5),
                    NavigationModifiers.builder()
                            .canOpenDoors(true)
                            .canPassDoors(true)
                            .build());

            runUntilFinished(env, controller, 600);

            assertEquals(NavigationState.COMPLETED, controller.state());
            assertEquals("true",
                    instance.getBlock(2, 40, 0).properties().get("open"));
            assertTrue(controller.nodes().stream().allMatch(
                    node -> node.graphZ() == 0));
            assertWaypointWasReachedBeforeAdvance(zombie, controller, 2);
            emit("zombie-temporary-door-capability", zombie, controller);
        }
    }

    @Test
    void pathIsRaisedWhenCauldronAppearsAfterPlanning(Env env) {
        Instance instance = preparedFlat(env);
        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature zombie = spawn(
                    instance, EntityType.ZOMBIE, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(zombie, service, 0.12);
            zombie.controller = controller;
            controller.moveTo(new Pos(5.5, 40, 0.5));

            assertTimeoutPreemptively(Duration.ofSeconds(5), () -> {
                Thread.sleep(20);
                while (service.activeCount() != 0 || service.queuedCount() != 0) {
                    Thread.sleep(1);
                }
            });
            // The completed search saw air here. Path acceptance must trim
            // against the current tick-thread world state.
            instance.setBlock(2, 40, 0, Block.CAULDRON);

            runUntilFinished(env, controller, 1_000);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + zombie.getPosition()
                            + ", nodes=" + controller.nodes());
            PathNode cauldron = controller.nodes().stream()
                    .filter(node -> node.graphX() == 2).findFirst().orElseThrow();
            PathNode following = controller.nodes().stream()
                    .filter(node -> node.graphX() == 3).findFirst().orElseThrow();
            assertEquals(41, cauldron.graphY());
            assertEquals(41, following.graphY());
            assertEquals(41, cauldron.y(), 1.0e-9);
            assertEquals(41, following.y(), 1.0e-9);
            assertTrue(zombie.positions.stream().mapToDouble(Pos::y)
                    .max().orElseThrow() > 40.5,
                    "entity never executed the raised cauldron waypoints");
            emit("zombie-dynamic-cauldron", zombie, controller);
        }
    }

    @Test
    void spiderFallbackClimbsVerticalObstacle(Env env) {
        Instance instance = preparedFlat(env);
        for (int y = 40; y <= 45; y++) {
            // Keep the wall wider than any incidental lateral steering so the
            // fixture measures climbing rather than a timing-sensitive edge
            // detour around one end.
            for (int z = -6; z <= 6; z++) instance.setBlock(3, y, z, Block.STONE);
        }

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature spider = spawn(instance, EntityType.SPIDER, new Pos(0.5, 40, 0.5));
            NavigationProfile profile = BuiltinNavigationProfiles
                    .forEntityType(EntityType.SPIDER);
            EntityNavigationController controller =
                    new EntityNavigationController(
                            spider, service, profile, 0.15);
            spider.controller = controller;
            Pos goal = new Pos(4.5, 46, 0.5);
            NavigationRequest meshRequest = new NavigationRequest(
                    instance, spider.getPosition(), goal,
                    spider.getBoundingBox(), profile, 16, 1, 1,
                    List.of(), 63, EntityTraversalState.GROUNDED, 16);
            controller.follow(AdaptiveMeshTestSupport.promoteAndReplay(
                    service, meshRequest));

            runUntilFinished(env, controller, 4_000);

            assertEquals(NavigationState.COMPLETED, controller.state());
            assertTrue(spider.getPosition().y() >= 45,
                    () -> "spider did not climb: " + spider.getPosition());
        }
    }

    @Test
    void verticalTargetExpandsSearchAndUsesDistantStaircaseWithoutJumpingBelowIt(
            Env env) {
        Instance instance = preparedFlat(env);
        Block stair = Block.STONE_STAIRS
                .withProperty("facing", "west")
                .withProperty("half", "bottom")
                .withProperty("shape", "straight");

        // The mob begins directly below the target. The only route travels 25
        // blocks away, climbs eight stairs, then returns on the upper walkway.
        for (int step = 0; step < 8; step++) {
            int x = 25 - step;
            for (int y = 40; y < 40 + step; y++) {
                instance.setBlock(x, y, 0, Block.STONE);
            }
            instance.setBlock(x, 40 + step, 0, stair);
        }
        for (int x = -1; x <= 17; x++) {
            instance.setBlock(x, 47, 0, Block.STONE);
        }
        for (int x = -2; x <= 28; x++) {
            for (int y = 40; y <= 51; y++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
        }

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(
                    instance, EntityType.ZOMBIE, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.16);
            mob.controller = controller;
            NavigationRequest proofRequest = NavigationRequest.builder(
                            instance, mob.getPosition(), new Pos(0.5, 48, 0.5),
                            mob.getBoundingBox(),
                            BuiltinNavigationProfiles.forEntity(mob))
                    .maxPathLength(128).nodeSearchRange(128)
                    .maxVisitedMultiplier(8).build();
            PathResult proof = new EntityPathfinder().findPath(
                    proofRequest, SearchControl.NONE);
            assertTrue(proof.found(),
                    () -> "distant staircase fixture has no route: " + proof);
            controller.moveTo(new Pos(0.5, 48, 0.5));

            runUntilFinished(env, controller, 3_000);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition()
                            + ", generation=" + controller.generation()
                            + ", recentPositions=" + mob.positions.stream()
                            .skip(Math.max(0, mob.positions.size() - 40L)).toList()
                            + ", nodes=" + controller.nodes());
            assertTrue(controller.generation() >= 2,
                    "the vertically trapped partial search was not expanded");
            assertTrue(mob.positions.stream().mapToDouble(Pos::x)
                            .max().orElseThrow() > 24,
                    () -> "mob never walked away to the staircase: "
                            + mob.positions);
            assertEquals(0, mob.upwardLaunches,
                    "distant ordinary stairs must not become jump launches");
            assertTrue(controller.nodes().stream().noneMatch(node ->
                            node.movement() == PathNode.Movement.JUMP),
                    () -> "vertical detour emitted jump nodes: "
                            + controller.nodes());
            assertTrue(mob.getPosition().y() >= 47.9,
                    () -> "mob stayed below the player: " + mob.getPosition());
            emit("vertical-distant-staircase", mob, controller);
        }
    }

    @Test
    void optInLadderRouteEntersClimbsAndExitsUsingLivePhysics(Env env) {
        Instance instance = preparedFlat(env);
        Block ladder = Block.LADDER.withProperty("facing", "west");
        for (int y = 40; y <= 44; y++) {
            instance.setBlock(1, y, 0, ladder);
            instance.setBlock(2, y, 0, Block.STONE);
        }
        instance.setBlock(3, 44, 0, Block.STONE);
        for (int y = 40; y <= 46; y++) {
            instance.setBlock(0, y, -1, Block.STONE);
            instance.setBlock(0, y, 1, Block.STONE);
            instance.setBlock(1, y, -1, Block.STONE);
            instance.setBlock(1, y, 1, Block.STONE);
        }
        NavigationProfile base = BuiltinNavigationProfiles.forEntityType(
                EntityType.ZOMBIE);
        NavigationProfile climbing = base.withGroundCapabilities(
                base.groundCapabilities().withClimbables(
                        ClimbableCapabilities.STANDARD));

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(instance, EntityType.ZOMBIE,
                    new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(mob, service, climbing, 0.16);
            mob.controller = controller;
            Pos climbGoal = new Pos(3.5, 45, 0.5);
            NavigationRequest meshRequest = new NavigationRequest(
                    instance, mob.getPosition(), climbGoal,
                    mob.getBoundingBox(), climbing, 16, 1, 1,
                    List.of(), 63, EntityTraversalState.GROUNDED, 16);
            controller.follow(AdaptiveMeshTestSupport.promoteAndReplay(
                    service, meshRequest));

            runUntilFinished(env, controller, 1_500);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition()
                            + ", ladderNodeIndex=" + controller.nodeIndex()
                            + ", ladderRecent=" + mob.positions.stream().skip(
                            Math.max(0, mob.positions.size() - 30L)).toList()
                            + ", nodes=" + controller.nodes());
            assertTrue(controller.nodes().stream().anyMatch(node ->
                            node.movement() == PathNode.Movement.CLIMB),
                    controller.nodes()::toString);
            assertTrue(mob.positions.stream().mapToDouble(Pos::y)
                            .max().orElseThrow() >= 44.9,
                    () -> "mob did not ascend ladder: " + mob.positions);
            assertTrue(mob.getPosition().x() > 2,
                    () -> "mob did not exit onto upper platform: "
                            + mob.getPosition());
            emit("opt-in-ladder-ascent", mob, controller);

            GroundCapabilities descentCapabilities = GroundCapabilities.builder().maxStepHeight(1).maxFallDistance(1).allowDiagonal(true).horizontalEdgeCost(false).platformJump(PlatformJumpCapabilities.DISABLED).climbables(ClimbableCapabilities.STANDARD).build();
            NavigationProfile descending =
                    base.withGroundCapabilities(descentCapabilities);
            ControlledCreature descendingMob = spawn(
                    instance, EntityType.ZOMBIE, new Pos(2.5, 45, 0.5));
            EntityNavigationController descendingController =
                    new EntityNavigationController(
                            descendingMob, service, descending, 0.16);
            descendingMob.controller = descendingController;
            descendingController.moveTo(new Pos(0.5, 40, 0.5));

            runUntilFinished(env, descendingController, 1_500);

            assertEquals(NavigationState.COMPLETED,
                    descendingController.state(),
                    () -> "descending position=" + descendingMob.getPosition()
                            + ", nodes=" + descendingController.nodes());
            assertTrue(descendingController.nodes().stream().anyMatch(node ->
                    node.movement() == PathNode.Movement.CLIMB));
            assertTrue(descendingController.nodes().stream().noneMatch(node ->
                            node.movement() == PathNode.Movement.FALL),
                    () -> "ladder descent became an uncontrolled fall: "
                            + descendingController.nodes());
            assertTrue(descendingMob.getPosition().y() < 40.6,
                    () -> "mob did not descend ladder: "
                            + descendingMob.getPosition());
            emit("opt-in-ladder-descent", descendingMob,
                    descendingController);

            ControlledCreature wideMob = spawn(instance, EntityType.ZOMBIE,
                    new Pos(-1, 40, 0.5));
            wideMob.setBoundingBox(1.8, 1.8, 0.8);
            EntityNavigationController wideController =
                    new EntityNavigationController(
                            wideMob, service, climbing, 0.16);
            wideMob.controller = wideController;
            wideController.moveTo(new Pos(3.5, 45, 0.5));

            runUntilFinished(env, wideController, 2_000);

            assertEquals(NavigationState.COMPLETED, wideController.state(),
                    () -> "wide position=" + wideMob.getPosition()
                            + ", nodes=" + wideController.nodes());
            assertTrue(wideController.nodes().stream().anyMatch(node ->
                    node.movement() == PathNode.Movement.CLIMB));
            assertTrue(wideMob.getPosition().y() >= 44.9,
                    () -> "wide mob did not climb safely: "
                            + wideMob.getPosition());
            emit("opt-in-wide-ladder-ascent", wideMob, wideController);
        }
    }

    @Test
    void everyNonLadderClimbableFamilyClimbsInLiveSimulation(Env env) {
        for (String id : List.of(
                "vine", "scaffolding",
                "weeping_vines", "weeping_vines_plant",
                "twisting_vines", "twisting_vines_plant",
                "cave_vines", "cave_vines_plant")) {
            Instance instance = preparedFlat(env);
            Block climbable = Block.fromKey(id);
            assertNotNull(climbable, id);
            for (int y = 40; y <= 44; y++) {
                instance.setBlock(1, y, 0, climbable);
            }
            instance.setBlock(2, 44, 0, Block.STONE);
            instance.setBlock(3, 44, 0, Block.STONE);
            NavigationProfile base = BuiltinNavigationProfiles.forEntityType(
                    EntityType.ZOMBIE);
            NavigationProfile climbing = NavigationProfile.builder(base.mode(), base.mobProfile(), base.groundCapabilities().withClimbables(
                            ClimbableCapabilities.STANDARD)).allowBreaching(base.allowBreaching()).prefersShallowWater(base.prefersShallowWater()).avoidSun(base.avoidSun()).directPathIgnoresFluids(base.directPathIgnoresFluids()).pathToTargetsBelowSurface(base.pathToTargetsBelowSurface()).build();

            try (AsyncEntityPathfindingService service =
                         new AsyncEntityPathfindingService(1, 8)) {
                ControlledCreature mob = spawn(instance, EntityType.ZOMBIE,
                        new Pos(0.5, 40, 0.5));
                EntityNavigationController controller =
                        new EntityNavigationController(
                                mob, service, climbing, 0.16);
                mob.controller = controller;
                controller.moveTo(new Pos(3.5, 45, 0.5));

                runUntilFinished(env, controller, 1_500);

                assertEquals(NavigationState.COMPLETED, controller.state(),
                        () -> id + " position=" + mob.getPosition()
                                + ", nodes=" + controller.nodes());
                assertTrue(controller.nodes().stream().anyMatch(node ->
                                node.movement() == PathNode.Movement.CLIMB),
                        () -> id + ": " + controller.nodes());
                assertTrue(mob.getPosition().y() >= 44.5,
                        () -> id + " did not climb: " + mob.getPosition());
                emit("opt-in-" + id + "-ascent", mob, controller);
            }
        }
    }

    @Test
    void sunAvoidingMobStopsBeforeLeavingRoof(Env env) {
        Instance instance = preparedFlat(env);
        for (int x = -2; x <= 3; x++) {
            for (int z = -2; z <= 2; z++) {
                instance.setBlock(x, 43, z, Block.STONE);
            }
        }

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(
                    instance, EntityType.ZOMBIE, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(8.5, 40, 0.5),
                    NavigationModifiers.builder().avoidSun(true).build());

            runUntilFinished(env, controller, 800);

            assertEquals(NavigationState.COMPLETED, controller.state());
            assertTrue(mob.getPosition().x() < 4.0,
                    () -> "sun-avoiding mob left cover: " + mob.getPosition());
            assertTrue(controller.nodes().stream().allMatch(
                    node -> node.graphX() <= 3),
                    () -> "accepted path contains exposed nodes: "
                            + controller.nodes());
            assertFalse(controller.nodes().isEmpty());
            emit("zombie-avoid-sun", mob, controller);
        }
    }

    private static Instance preparedFlat(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 3,
                (x, z) -> instance.loadChunk(x, z).join());
        return instance;
    }

    private static ControlledCreature spawn(Instance instance, EntityType type, Pos position) {
        ControlledCreature creature = new ControlledCreature(type);
        creature.setInstance(instance, position).join();
        return creature;
    }

    private static NavigationProfile jumpProfile(double distance) {
        NavigationProfile base =
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
        return base.withGroundCapabilities(
                base.groundCapabilities().withPlatformJump(
                        PlatformJumpCapabilities.builder().maxHorizontalDistance(distance).maxRise(0).maxDrop(0).apexClearance(1).build()));
    }

    private static void runUntilFinished(Env env, EntityNavigationController controller,
                                         int maximumTicks) {
        assertTimeoutPreemptively(Duration.ofSeconds(10), () -> {
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

    private static double graphLength(List<PathNode> nodes) {
        double length = 0;
        for (int i = 1; i < nodes.size(); i++) {
            PathNode a = nodes.get(i - 1);
            PathNode b = nodes.get(i);
            int dx = b.graphX() - a.graphX();
            int dy = b.graphY() - a.graphY();
            int dz = b.graphZ() - a.graphZ();
            length += Math.sqrt(dx * dx + dy * dy + dz * dz);
        }
        return length;
    }

    private static double traveled(List<Pos> positions) {
        double result = 0;
        for (int i = 1; i < positions.size(); i++) {
            result += positions.get(i - 1).distance(positions.get(i));
        }
        return result;
    }

    private static void assertTrajectoryDoesNotIntersectWall(
            ControlledCreature creature) {
        for (Pos position : creature.positions) {
            for (int x = -5; x <= 5; x++) {
                for (int y = 40; y <= 41; y++) {
                    Pos blockPosition = new Pos(x, y, 5);
                    assertFalse(Block.STONE.collisionShape().intersectBox(
                                    position.sub(blockPosition),
                                    creature.getBoundingBox()),
                            () -> "trajectory clipped wall at entity=" + position
                                    + ", block=" + blockPosition);
                }
            }
        }
    }

    private static void assertTrajectoryAvoidsOpeningWall(
            ControlledCreature creature) {
        for (Pos position : creature.positions) {
            for (int z = -5; z <= 5; z++) {
                for (int y = 40; y <= 42; y++) {
                    if (z == 0 && y < 42) continue;
                    Pos block = new Pos(3, y, z);
                    assertFalse(Block.STONE.collisionShape()
                                    .intersectBox(position.sub(block),
                                            creature.getBoundingBox()),
                            () -> "custom bounding box clipped wall at entity="
                                    + position + ", block=" + block);
                }
            }
        }
    }

    private static void assertTrajectoryRemainsSupported(
            ControlledCreature creature) {
        Instance instance = creature.getInstance();
        assertNotNull(instance);
        double radiusX = creature.getBoundingBox().width() * 0.5;
        double radiusZ = creature.getBoundingBox().depth() * 0.5;
        for (Pos position : creature.positions) {
            int supportY = (int) Math.floor(position.y() - 1.0e-5);
            int minimumX = (int) Math.floor(position.x() - radiusX + 1.0e-5);
            int maximumX = (int) Math.floor(position.x() + radiusX - 1.0e-5);
            int minimumZ = (int) Math.floor(position.z() - radiusZ + 1.0e-5);
            int maximumZ = (int) Math.floor(position.z() + radiusZ - 1.0e-5);
            boolean supported = false;
            for (int x = minimumX; x <= maximumX && !supported; x++) {
                for (int z = minimumZ; z <= maximumZ; z++) {
                    if (instance.getBlock(x, supportY, z).collisionShape().relativeEnd().y() >= 1) {
                        supported = true;
                        break;
                    }
                }
            }
            assertTrue(supported,
                    () -> "entity had no supporting block under its footprint at "
                            + position);
        }
    }

    private static void assertTrajectoryCornersHaveNearbySupport(
            ControlledCreature creature, double maximumDrop) {
        Instance instance = creature.getInstance();
        assertNotNull(instance);
        double radiusX = creature.getBoundingBox().width() * 0.5 - 0.05;
        double radiusZ = creature.getBoundingBox().depth() * 0.5 - 0.05;
        for (Pos position : creature.positions) {
            for (double offsetX : new double[]{-radiusX, radiusX}) {
                for (double offsetZ : new double[]{-radiusZ, radiusZ}) {
                    int x = (int) Math.floor(position.x() + offsetX);
                    int z = (int) Math.floor(position.z() + offsetZ);
                    int upperY = (int) Math.floor(position.y() - 1.0e-5);
                    boolean supported = false;
                    for (int y = upperY; y >= upperY - 2; y--) {
                        Block block = instance.getBlock(x, y, z);
                        double supportHeight = y + block.collisionShape().relativeEnd().y();
                        if (block.collisionShape().relativeEnd().y() > 0
                                && position.y() - supportHeight
                                <= maximumDrop + 1.0e-5) {
                            supported = true;
                            break;
                        }
                    }
                    double cornerX = position.x() + offsetX;
                    double cornerZ = position.z() + offsetZ;
                    assertTrue(supported,
                            () -> "large-mob footprint corner lacked nearby support at "
                                    + position + " corner=(" + cornerX + ","
                                    + cornerZ + ")");
                }
            }
        }
    }

    private static void addWallWithGap(
            Instance instance, List<Pos> wallBlocks, int x,
            int minimumZ, int maximumZ, int gapMinimumZ, int gapMaximumZ) {
        for (int z = minimumZ; z <= maximumZ; z++) {
            if (z >= gapMinimumZ && z <= gapMaximumZ) continue;
            for (int y = 40; y <= 41; y++) {
                instance.setBlock(x, y, z, Block.STONE);
                wallBlocks.add(new Pos(x, y, z));
            }
        }
    }

    private static void assertGapUsed(
            List<PathNode> nodes, int wallX,
            int gapMinimumZ, int gapMaximumZ) {
        PathNode crossing = nodes.stream()
                .filter(node -> node.graphX() == wallX)
                .findFirst()
                .orElseThrow(() -> new AssertionError(
                        "path never crossed wall x=" + wallX + ": " + nodes));
        assertTrue(crossing.graphZ() >= gapMinimumZ
                        && crossing.graphZ() <= gapMaximumZ,
                () -> "path crossed wall outside gap: " + crossing);
    }

    private static void assertTrajectoryUsedGap(
            ControlledCreature creature, int wallX,
            int gapMinimumZ, int gapMaximumZ) {
        Pos crossing = creature.positions.stream()
                .filter(position -> Math.abs(position.x() - (wallX + 1)) < 0.6)
                .min(java.util.Comparator.comparingDouble(position ->
                        Math.abs(position.x() - (wallX + 1))))
                .orElseThrow(() -> new AssertionError(
                        "trajectory never crossed wall x=" + wallX));
        assertTrue(crossing.z() >= gapMinimumZ
                        && crossing.z() < gapMaximumZ + 1,
                () -> "trajectory crossed wall outside gap: " + crossing);
    }

    private static void assertTrajectoryAvoidsBlocks(
            ControlledCreature creature, List<Pos> blocks) {
        for (Pos position : creature.positions) {
            for (Pos block : blocks) {
                assertFalse(Block.STONE.collisionShape()
                                .intersectBox(position.sub(block),
                                        creature.getBoundingBox()),
                        () -> "trajectory clipped block at entity="
                                + position + ", block=" + block);
            }
        }
    }

    /**
     * Sweeps the recorded trajectory against the state each watched block
     * actually held on that tick, and requires the entity to have occupied
     * every watched cell at least once. Both halves matter: a follower that
     * routed around the block would satisfy the sweep vacuously.
     */
    private static void assertNeverOverlappedAClosedOpenable(
            ControlledCreature creature, String label) {
        assertFalse(creature.blockFrames.isEmpty(),
                () -> label + ": no block states were sampled");
        boolean[] entered = new boolean[creature.watchedBlocks.size()];
        for (int tick = 0; tick < creature.blockFrames.size(); tick++) {
            BlockFrame frame = creature.blockFrames.get(tick);
            for (int i = 0; i < creature.watchedBlocks.size(); i++) {
                Pos block = creature.watchedBlocks.get(i);
                Block state = frame.states().get(i);
                int sample = tick;
                for (Pos position : List.of(frame.before(), frame.after())) {
                    if (Block.STONE.collisionShape().intersectBox(
                            position.sub(block), creature.getBoundingBox())) {
                        entered[i] = true;
                    }
                    if (BlockTraversalData.isOpen(state)) continue;
                    assertFalse(state.collisionShape().intersectBox(
                                    position.sub(block),
                                    creature.getBoundingBox()),
                            () -> label + ": entity overlapped closed "
                                    + state.key().value() + " at tick " + sample
                                    + ", entity=" + position
                                    + ", block=" + block);
                }
            }
        }
        for (int i = 0; i < entered.length; i++) {
            int index = i;
            assertTrue(entered[i], () -> label + ": entity never occupied "
                    + creature.watchedBlocks.get(index));
        }
    }

    private static void assertWaypointWasReachedBeforeAdvance(
            ControlledCreature creature, EntityNavigationController controller,
            int graphX) {
        int nodeIndex = -1;
        for (int i = 0; i < controller.nodes().size(); i++) {
            if (controller.nodes().get(i).graphX() == graphX) {
                nodeIndex = i;
                break;
            }
        }
        int expectedIndex = nodeIndex;
        assertTrue(expectedIndex >= 0, "fixture path lacks expected node");
        Advancement transition = creature.advancements.stream()
                .filter(value -> value.before == expectedIndex
                        && value.after == expectedIndex + 1)
                .findFirst().orElseThrow(
                        () -> new AssertionError("node was never advanced: "
                                + controller.nodes()));
        PathNode node = controller.nodes().get(expectedIndex);
        double tolerance = creature.getBoundingBox().width() > 0.75
                ? creature.getBoundingBox().width() / 2
                : 0.75 - creature.getBoundingBox().width() / 2;
        assertTrue(Math.abs(transition.position.x() - node.x()) < tolerance
                        && Math.abs(transition.position.z() - node.z()) < tolerance,
                () -> "protected waypoint was skipped at " + transition.position
                        + ", node=" + node + ", tolerance=" + tolerance);
    }

    private static void emit(String label, ControlledCreature creature,
                             EntityNavigationController controller) {
        double traveled = 0;
        double minX = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < creature.positions.size(); i++) {
            Pos position = creature.positions.get(i);
            minX = Math.min(minX, position.x());
            maxX = Math.max(maxX, position.x());
            if (i > 0) traveled += creature.positions.get(i - 1).distance(position);
        }
        System.out.printf(java.util.Locale.ROOT,
                "GROUND-E2E %s state=%s samples=%d traveled=%.6f "
                        + "x=[%.3f,%.3f] final=%s graphLength=%.9f nodes=%s%n",
                label, controller.state(), creature.positions.size(), traveled,
                minX, maxX, creature.getPosition(),
                graphLength(controller.nodes()), controller.nodes());
    }

    private static final class ControlledCreature extends EntityCreature {
        private EntityNavigationController controller;
        private final List<Pos> positions = new ArrayList<>();
        private final List<Advancement> advancements = new ArrayList<>();
        private final List<BlockFrame> blockFrames = new ArrayList<>();
        private List<Pos> watchedBlocks = List.of();
        private int upwardLaunches;

        private ControlledCreature(EntityType entityType) {
            super(entityType);
        }

        @Override
        public void update(long time) {
            Pos beforePosition = getPosition();
            if (controller != null) {
                int before = controller.nodeIndex();
                double beforeVerticalVelocity = getVelocity().y();
                controller.tick();
                if (beforeVerticalVelocity <= 0
                        && getVelocity().y() >= 4) {
                    upwardLaunches++;
                }
                int after = controller.nodeIndex();
                if (after != before) {
                    advancements.add(new Advancement(before, after, beforePosition));
                }
            }
            // Sampled between the controller and the physics step, which is
            // the state Minestom moves the entity through on this tick.
            List<Block> states = sampleWatchedBlocks();
            super.update(time);
            positions.add(getPosition());
            if (!watchedBlocks.isEmpty()) {
                blockFrames.add(new BlockFrame(
                        beforePosition, getPosition(), states));
            }
        }

        private List<Block> sampleWatchedBlocks() {
            if (watchedBlocks.isEmpty()) return List.of();
            Instance instance = getInstance();
            List<Block> states = new ArrayList<>(watchedBlocks.size());
            for (Pos block : watchedBlocks) {
                states.add(instance.getBlock(
                        block.blockX(), block.blockY(), block.blockZ()));
            }
            return List.copyOf(states);
        }
    }

    private record Advancement(int before, int after, Pos position) {
    }

    /**
     * One tick of the trajectory beside the states the watched blocks held on
     * that tick. A block whose shape changes mid-run cannot be swept against
     * its final state, so the state is recorded with the positions it applied
     * to rather than read once at the end.
     */
    private record BlockFrame(Pos before, Pos after, List<Block> states) {
    }
}
