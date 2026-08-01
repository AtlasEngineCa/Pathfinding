package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.result.PathNode;
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
import java.util.SplittableRandom;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Reproducible generated Minestom simulations. Every failure includes the
 * complete seed and compact fixture description, so it can be promoted into a
 * minimal permanent regression without guessing at timing or random state.
 */
@EnvTest
class GeneratedNavigatorE2ETest {
    /**
     * CI keeps a compact permanent corpus. Set PATHFINDING_FUZZ_CASES to run
     * a broader deterministic prefix locally or in a nightly workflow.
     */
    private static final int CASES_PER_FAMILY = Math.max(1,
            Integer.parseInt(System.getenv().getOrDefault(
                    "PATHFINDING_FUZZ_CASES", "12")));
    private static final long ROOT_SEED = 0x6E61762D66757A7AL;

    private static final EntityType[] GROUND_TYPES = {
            EntityType.SILVERFISH, EntityType.ZOMBIE,
            EntityType.CAMEL, EntityType.RAVAGER
    };
    private static final EntityType[] WATER_TYPES = {
            EntityType.COD, EntityType.DOLPHIN,
            EntityType.GUARDIAN, EntityType.ELDER_GUARDIAN
    };
    private static final EntityType[] FLYING_TYPES = {
            EntityType.BEE, EntityType.ALLAY, EntityType.PARROT,
            EntityType.WITHER
    };
    private static final EntityType[] AMPHIBIOUS_TYPES = {
            EntityType.AXOLOTL, EntityType.DROWNED,
            EntityType.FROG, EntityType.TURTLE
    };

    @Test
    void generatedGroundDetoursRemainLiveAndCollisionFree(Env env) {
        Instance instance = prepared(env);
        try (var service = new AsyncEntityPathfindingService(2, 32)) {
            assertAll("generated ground cases", cases("ground", index -> {
                long seed = seed(0, index);
                SplittableRandom random = new SplittableRandom(seed);
                reset(instance);
                EntityType type = pick(random, GROUND_TYPES);
                int halfWall = random.nextInt(1, 4);
                int wallHeight = random.nextInt(2, 6);
                double startZ = random.nextDouble(-0.35, 0.35);
                List<Pos> solids = new ArrayList<>();
                for (int y = 40; y < 40 + wallHeight; y++) {
                    for (int z = -halfWall; z <= halfWall; z++) {
                        solid(instance, solids, 4, y, z);
                    }
                }
                String fixture = "type=" + type.key().asString()
                        + ", wallHalf=" + halfWall + ", wallHeight="
                        + wallHeight + ", startZ=" + startZ;
                runCase(env, instance, service, type,
                        new Pos(0.5, 40, 0.5 + startZ),
                        new Pos(9.5, 40, 0.5), solids,
                        NavigationMode.GROUND, seed, fixture);
            }));
        }
    }

    @Test
    void generatedSwimDetoursRespectCompleteBodyVolume(Env env) {
        Instance instance = prepared(env);
        try (var service = new AsyncEntityPathfindingService(2, 32)) {
            assertAll("generated water cases", cases("water", index -> {
                long seed = seed(1, index);
                SplittableRandom random = new SplittableRandom(seed);
                reset(instance);
                fill(instance, -4, 13, 39, 48, -8, 8, Block.WATER);
                EntityType type = pick(random, WATER_TYPES);
                int halfDepth = random.nextInt(0, 3);
                int lowY = random.nextInt(39, 42);
                int highY = random.nextInt(45, 49);
                double startZ = random.nextDouble(-0.3, 0.3);
                List<Pos> solids = new ArrayList<>();
                for (int y = lowY; y <= highY; y++) {
                    for (int z = -halfDepth; z <= halfDepth; z++) {
                        solid(instance, solids, 4, y, z);
                    }
                }
                String fixture = "type=" + type.key().asString()
                        + ", plugZ=" + halfDepth + ", plugY=" + lowY
                        + ".." + highY + ", startZ=" + startZ;
                runCase(env, instance, service, type,
                        new Pos(0.5, 42.5, 0.5 + startZ),
                        new Pos(9.5, 45.5, 0.5), solids,
                        NavigationMode.WATER, seed, fixture);
            }));
        }
    }

    @Test
    void generatedFlightDetoursRespectCompleteBodyVolume(Env env) {
        Instance instance = prepared(env);
        try (var service = new AsyncEntityPathfindingService(2, 32)) {
            assertAll("generated flying cases", cases("flying", index -> {
                long seed = seed(2, index);
                SplittableRandom random = new SplittableRandom(seed);
                reset(instance);
                EntityType type = pick(random, FLYING_TYPES);
                int halfDepth = random.nextInt(0, 3);
                int lowY = random.nextInt(40, 44);
                int highY = random.nextInt(48, 54);
                double startZ = random.nextDouble(-0.3, 0.3);
                List<Pos> solids = new ArrayList<>();
                for (int y = lowY; y <= highY; y++) {
                    for (int z = -halfDepth; z <= halfDepth; z++) {
                        solid(instance, solids, 4, y, z);
                    }
                }
                String fixture = "type=" + type.key().asString()
                        + ", wallZ=" + halfDepth + ", wallY=" + lowY
                        + ".." + highY + ", startZ=" + startZ;
                runCase(env, instance, service, type,
                        new Pos(0.5, 45.5, 0.5 + startZ),
                        new Pos(9.5, 48.5, 0.5), solids,
                        NavigationMode.FLYING, seed, fixture);
            }));
        }
    }

    @Test
    void generatedAmphibiousTransitionsDoNotFreezeAtEitherShore(Env env) {
        Instance instance = prepared(env);
        try (var service = new AsyncEntityPathfindingService(2, 32)) {
            assertAll("generated amphibious cases", cases("amphibious", index -> {
                long seed = seed(3, index);
                SplittableRandom random = new SplittableRandom(seed);
                reset(instance);
                EntityType type = pick(random, AMPHIBIOUS_TYPES);
                int startX = random.nextInt(2, 4);
                int endX = random.nextInt(6, 8);
                int halfWidth = random.nextInt(2, 5);
                int depth = random.nextInt(1, 4);
                for (int x = startX; x <= endX; x++) {
                    for (int z = -halfWidth; z <= halfWidth; z++) {
                        for (int y = 40 - depth; y <= 40; y++) {
                            instance.setBlock(x, y, z, Block.WATER);
                        }
                    }
                }
                String fixture = "type=" + type.key().asString()
                        + ", pondX=" + startX + ".." + endX
                        + ", pondHalfZ=" + halfWidth + ", depth=" + depth;
                runCase(env, instance, service, type,
                        new Pos(0.5, 40, 0.5),
                        new Pos(10.5, 40, 0.5), List.of(),
                        NavigationMode.AMPHIBIOUS, seed, fixture);
            }));
        }
    }

    @Test
    void generatedMidRouteBlockChangesReplanBeforeCollision(Env env) {
        Instance instance = prepared(env);
        try (var service = new AsyncEntityPathfindingService(2, 32)) {
            assertAll("generated mutation cases", cases("mutation", index -> {
                long seed = seed(4, index);
                SplittableRandom random = new SplittableRandom(seed);
                reset(instance);
                EntityType type = pick(random, new EntityType[]{
                        EntityType.SILVERFISH, EntityType.ZOMBIE,
                        EntityType.RAVAGER});
                int mutationTick = random.nextInt(1, 24);
                int wallX = random.nextInt(4, 7);
                int halfWall = random.nextInt(1, 3);
                double startZ = random.nextDouble(-0.3, 0.3);
                String fixture = "type=" + type.key().asString()
                        + ", mutationTick=" + mutationTick + ", wallX="
                        + wallX + ", wallHalf=" + halfWall
                        + ", startZ=" + startZ;
                runMutationCase(env, instance, service, type,
                        new Pos(0.5, 40, 0.5 + startZ),
                        new Pos(11.5, 40, 0.5), wallX, halfWall,
                        mutationTick, seed, fixture);
            }));
        }
    }

    @Test
    void generatedGuaranteedMazesRemainSolvable(Env env) {
        Instance instance = prepared(env);
        try (var service = new AsyncEntityPathfindingService(2, 32)) {
            assertAll("generated solvable mazes", cases("maze", index -> {
                long seed = seed(5, index);
                SplittableRandom random = new SplittableRandom(seed);
                reset(instance);
                EntityType type = pick(random, new EntityType[]{
                        EntityType.SILVERFISH, EntityType.ZOMBIE});
                List<Pos> solids = new ArrayList<>();
                int[] gates = new int[3];
                int previous = random.nextBoolean() ? -3 : 3;
                for (int wall = 0; wall < gates.length; wall++) {
                    int gate = wall == 0 ? previous : -previous;
                    if (random.nextInt(4) == 0) gate += random.nextBoolean() ? 1 : -1;
                    gate = Math.max(-3, Math.min(3, gate));
                    gates[wall] = gate;
                    previous = gate;
                    int x = 3 + wall * 3;
                    for (int z = -5; z <= 5; z++) {
                        if (z == gate) continue;
                        for (int y = 40; y <= 43; y++) solid(instance, solids, x, y, z);
                    }
                }
                String fixture = "type=" + type.key().asString()
                        + ", gates=" + java.util.Arrays.toString(gates)
                        + ", blocks=" + compactBlocks(solids);
                runCase(env, instance, service, type,
                        new Pos(0.5, 40, 0.5), new Pos(12.5, 40, 0.5),
                        solids, NavigationMode.GROUND, seed, fixture);
            }));
        }
    }

    @Test
    void generatedBoundingBoxThresholdsFitClearanceOpenings(Env env) {
        Instance instance = prepared(env);
        double[] thresholds = {0.499, 0.501, 0.999, 1.001, 1.499, 1.999};
        try (var service = new AsyncEntityPathfindingService(2, 32)) {
            assertAll("generated bounding-box thresholds", cases("bounds", index -> {
                long seed = seed(6, index);
                SplittableRandom random = new SplittableRandom(seed);
                reset(instance);
                double width = thresholds[random.nextInt(thresholds.length)];
                double depth = thresholds[random.nextInt(thresholds.length)];
                double height = random.nextBoolean() ? 1.799 : 2.001;
                int opening = (int) Math.ceil(depth);
                double centerZ = opening * 0.5;
                List<Pos> solids = new ArrayList<>();
                for (int z = -5; z <= 6; z++) {
                    if (z >= 0 && z < opening) continue;
                    for (int y = 40; y <= 44; y++) solid(instance, solids, 5, y, z);
                }
                String fixture = "bbox=" + width + "x" + height + "x" + depth
                        + ", opening=" + opening + ", blocks=" + compactBlocks(solids);
                runShapedCase(env, instance, service,
                        new Pos(0.5, 40, centerZ), new Pos(10.5, 40, centerZ),
                        width, height, depth, solids, seed, fixture);
            }));
        }
    }

    @Test
    void generatedMultipleWorldChangesRemainLive(Env env) {
        Instance instance = prepared(env);
        try (var service = new AsyncEntityPathfindingService(2, 32)) {
            assertAll("generated multi-event cases", cases("multi-event", index -> {
                long seed = seed(7, index);
                SplittableRandom random = new SplittableRandom(seed);
                reset(instance);
                int firstTick = random.nextInt(2, 12);
                int secondTick = firstTick + random.nextInt(8, 24);
                String fixture = "firstTick=" + firstTick + ", secondTick=" + secondTick;
                runMultiMutationCase(env, instance, service, firstTick, secondTick,
                        seed, fixture);
            }));
        }
    }

    @Test
    void generatedStatefulWorldAndTargetSchedulesRemainLive(Env env) {
        Instance instance = prepared(env);
        try (var service = new AsyncEntityPathfindingService(1, 4)) {
            assertAll("generated stateful schedules", cases("stateful", index -> {
                long seed = seed(11, index);
                SplittableRandom random = new SplittableRandom(seed);
                reset(instance);
                int firstTick = random.nextInt(2, 10);
                int wallTick = firstTick + random.nextInt(3, 10);
                int releaseTick = wallTick + random.nextInt(4, 14);
                int side = random.nextBoolean() ? -3 : 3;
                int wallX = random.nextInt(5, 8);
                instance.setBlock(2, 40, 0, Block.STONE_SLAB
                        .withProperty("type", "bottom"));
                instance.setBlock(3, 40, 0, Block.OAK_STAIRS
                        .withProperty("facing", random.nextBoolean() ? "east" : "west")
                        .withProperty("half", "bottom")
                        .withProperty("shape", "straight"));
                String fixture = "firstTick=" + firstTick + ", wallTick="
                        + wallTick + ", releaseTick=" + releaseTick
                        + ", side=" + side + ", wallX=" + wallX;
                runStatefulSchedule(env, instance, service,
                        firstTick, wallTick, releaseTick, side, wallX,
                        seed, fixture);
            }));
        }
    }

    @Test
    void mirroredMazesPreserveReachability(Env env) {
        Instance instance = prepared(env);
        try (var service = new AsyncEntityPathfindingService(2, 32)) {
            assertAll("mirrored maze metamorphism", cases("mirror", index -> {
                long seed = seed(8, index);
                SplittableRandom random = new SplittableRandom(seed);
                int side = random.nextBoolean() ? 2 : -2;
                int[] gates = {side, -side, side};
                for (int sign : new int[]{1, -1}) {
                    reset(instance);
                    List<Pos> solids = new ArrayList<>();
                    for (int wall = 0; wall < gates.length; wall++) {
                        int x = 3 + wall * 3;
                        int gate = gates[wall] * sign;
                        for (int z = -5; z <= 5; z++) {
                            if (z == gate) continue;
                            for (int y = 40; y <= 43; y++) {
                                solid(instance, solids, x, y, z);
                            }
                        }
                    }
                    String fixture = "mirror=" + sign + ", gates="
                            + java.util.Arrays.toString(gates)
                            + ", reducedBlocks=" + compactBlocks(solids);
                    runCase(env, instance, service, EntityType.SILVERFISH,
                            new Pos(0.5, 40, 0.5), new Pos(12.5, 40, 0.5),
                            solids, NavigationMode.GROUND, seed, fixture);
                }
            }));
        }
    }

    @Test
    void extremeGateSequenceContinuesAcrossPartialRoutes(Env env) {
        Instance instance = prepared(env);
        try (var service = new AsyncEntityPathfindingService(2, 32)) {
            for (int repetition = 0; repetition < 24; repetition++) {
                reset(instance);
                List<Pos> solids = new ArrayList<>();
                int[] gates = {1, -3, 3};
                for (int wall = 0; wall < gates.length; wall++) {
                    int x = 3 + wall * 3;
                    for (int z = -5; z <= 5; z++) {
                        if (z == gates[wall]) continue;
                        for (int y = 40; y <= 43; y++) {
                            solid(instance, solids, x, y, z);
                        }
                    }
                }
                runCase(env, instance, service, EntityType.SILVERFISH,
                        new Pos(0.5, 40, 0.5), new Pos(12.5, 40, 0.5), solids,
                        NavigationMode.GROUND, 0x67ccf007ea2c0f69L,
                        "repetition=" + repetition
                                + ", gates=[1,-3,3], reducedBlocks="
                                + compactBlocks(solids));
            }
        }
    }

    @Test
    void generatedMixedCollisionShapesRemainLive(Env env) {
        Instance instance = prepared(env);
        try (var service = new AsyncEntityPathfindingService(2, 32)) {
            assertAll("generated mixed-shape cases", cases("mixed-shapes", index -> {
                long seed = seed(9, index);
                SplittableRandom random = new SplittableRandom(seed);
                reset(instance);
                instance.setBlock(2, 40, 0, Block.STONE_SLAB
                        .withProperty("type", "bottom"));
                instance.setBlock(3, 40, 0, Block.OAK_STAIRS
                        .withProperty("facing", random.nextBoolean() ? "east" : "west")
                        .withProperty("half", "bottom")
                        .withProperty("shape", "straight"));
                instance.setBlock(4, 40, 0, Block.SNOW.withProperty(
                        "layers", String.valueOf(random.nextInt(2, 6))));
                List<Pos> solids = new ArrayList<>();
                int gate = random.nextBoolean() ? -2 : 2;
                for (int z = -4; z <= 4; z++) {
                    if (z == gate) continue;
                    for (int y = 40; y <= 43; y++) solid(instance, solids, 7, y, z);
                }
                String fixture = "gate=" + gate + ", snowLayers="
                        + instance.getBlock(4, 40, 0).getProperty("layers")
                        + ", reducedBlocks=" + compactBlocks(solids);
                runCase(env, instance, service, EntityType.ZOMBIE,
                        new Pos(0.5, 40, 0.5), new Pos(11.5, 40, 0.5),
                        solids, NavigationMode.GROUND, seed, fixture);
            }));
        }
    }

    @Test
    void concurrentRetargetingUnderSingleWorkerRemainsLive(Env env) throws Exception {
        Instance instance = prepared(env);
        reset(instance);
        try (var service = new AsyncEntityPathfindingService(1, 64)) {
            List<TrackedCreature> mobs = new ArrayList<>();
            List<EntityNavigationController> controllers = new ArrayList<>();
            try {
                for (int index = 0; index < 8; index++) {
                    double z = -7.5 + index * 2;
                    TrackedCreature mob = new TrackedCreature(
                            index % 2 == 0 ? EntityType.ZOMBIE : EntityType.SILVERFISH);
                    mob.setInstance(instance, new Pos(0.5, 40, z)).join();
                    EntityNavigationController controller =
                            EntityNavigationController.builtin(mob, service, 0.18);
                    mob.controller = controller;
                    controller.moveTo(new Pos(12.5, 40, z));
                    mobs.add(mob);
                    controllers.add(controller);
                }
                for (int tick = 0; tick < 2_000; tick++) {
                    if (tick == 12) {
                        for (int index = 0; index < controllers.size(); index++) {
                            double z = -7.5 + index * 2;
                            controllers.get(index).moveTo(new Pos(2.5, 40, z));
                        }
                    } else if (tick == 24) {
                        for (int index = 0; index < controllers.size(); index++) {
                            double z = -7.5 + index * 2;
                            controllers.get(index).moveTo(new Pos(12.5, 40, z));
                        }
                    }
                    env.tick();
                    if (controllers.stream().anyMatch(controller ->
                            controller.state() == NavigationState.COMPUTING)) {
                        Thread.sleep(1);
                    }
                    if (tick > 24 && controllers.stream().allMatch(controller ->
                            controller.state() == NavigationState.COMPLETED)) break;
                }
                for (int index = 0; index < controllers.size(); index++) {
                    int mobIndex = index;
                    assertEquals(NavigationState.COMPLETED,
                            controllers.get(index).state(),
                            () -> "concurrent mob " + mobIndex + " ended "
                                    + controllers.get(mobIndex).state() + " at "
                                    + mobs.get(mobIndex).getPosition());
                    assertNoLongFreeze(mobs.get(index).positions, 240,
                            "concurrent mob=" + index);
                }
            } finally {
                mobs.forEach(TrackedCreature::remove);
            }
        }
    }

    @Test
    void saturatedControllersSurviveRemovalRetargetAndBlockChange(Env env)
            throws Exception {
        Instance instance = prepared(env);
        reset(instance);
        try (var service = new AsyncEntityPathfindingService(1, 2)) {
            List<TrackedCreature> mobs = new ArrayList<>();
            List<EntityNavigationController> controllers = new ArrayList<>();
            boolean[] removed = new boolean[24];
            try {
                for (int index = 0; index < 24; index++) {
                    double z = -8.5 + index * (17.0 / 23.0);
                    TrackedCreature mob = new TrackedCreature(EntityType.SILVERFISH);
                    mob.setInstance(instance, new Pos(0.5, 40, z)).join();
                    EntityNavigationController controller =
                            EntityNavigationController.builtin(mob, service, 0.2);
                    mob.controller = controller;
                    controller.moveTo(new Pos(13.5, 40, z));
                    mobs.add(mob);
                    controllers.add(controller);
                }
                for (int tick = 0; tick < 3_000; tick++) {
                    if (tick == 5) {
                        for (int index = 0; index < mobs.size(); index += 4) {
                            removed[index] = true;
                            mobs.get(index).remove();
                        }
                    }
                    if (tick == 10 || tick == 18) {
                        double targetX = tick == 10 ? 3.5 : 13.5;
                        for (int index = 0; index < controllers.size(); index++) {
                            if (removed[index]) continue;
                            double z = -8.5 + index * (17.0 / 23.0);
                            controllers.get(index).moveTo(new Pos(targetX, 40, z));
                        }
                    }
                    if (tick == 26) {
                        for (int y = 40; y <= 43; y++) {
                            for (int z = -8; z <= 8; z++) {
                                if (z == -6 || z == 0 || z == 6) continue;
                                instance.setBlock(7, y, z, Block.STONE);
                            }
                        }
                        for (int index = 0; index < controllers.size(); index++) {
                            if (!removed[index]) controllers.get(index)
                                    .onBlockChanged(new Pos(7, 40, 0));
                        }
                    }
                    env.tick();
                    if (tick > 26 && allSurvivorsCompleted(controllers, removed)) break;
                    if (tick < 80 || tick % 4 == 0) Thread.sleep(1);
                }
                for (int index = 0; index < controllers.size(); index++) {
                    if (removed[index]) continue;
                    int mobIndex = index;
                    assertEquals(NavigationState.COMPLETED,
                            controllers.get(index).state(),
                            () -> "saturated survivor=" + mobIndex + ", state="
                                    + controllers.get(mobIndex).state() + ", pos="
                                    + mobs.get(mobIndex).getPosition());
                    assertNoLongFreeze(mobs.get(index).positions, 360,
                            "saturated survivor=" + index);
                }
            } finally {
                for (int index = 0; index < mobs.size(); index++) {
                    if (!removed[index]) mobs.get(index).remove();
                }
            }
        }
    }

    @Test
    void generatedFlyingBoundingBoxesCrossThreeDimensionalApertures(Env env) {
        Instance instance = prepared(env);
        double[] thresholds = {0.499, 0.999, 1.001, 1.499, 1.999};
        try (var service = new AsyncEntityPathfindingService(2, 32)) {
            assertAll("generated flying apertures", cases("flying-aperture", index -> {
                long seed = seed(10, index);
                SplittableRandom random = new SplittableRandom(seed);
                reset(instance);
                double width = thresholds[random.nextInt(thresholds.length)];
                double height = thresholds[random.nextInt(thresholds.length)];
                double depth = thresholds[random.nextInt(thresholds.length)];
                int openingHeight = (int) Math.ceil(height);
                int openingDepth = (int) Math.ceil(depth);
                int openingY = 43;
                double centerZ = openingDepth * 0.5;
                List<Pos> solids = new ArrayList<>();
                for (int y = 40; y <= 50; y++) {
                    for (int z = -4; z <= 5; z++) {
                        boolean aperture = y >= openingY
                                && y < openingY + openingHeight
                                && z >= 0 && z < openingDepth;
                        if (!aperture) solid(instance, solids, 5, y, z);
                    }
                }
                runShapedSpatialCase(env, instance, service,
                        new Pos(0.5, openingY, centerZ),
                        new Pos(10.5, openingY, centerZ), width, height, depth,
                        solids, seed, "bbox=" + width + "x" + height + "x"
                                + depth + ", aperture=" + openingHeight + "x"
                                + openingDepth);
            }));
        }
    }

    private static boolean allSurvivorsCompleted(
            List<EntityNavigationController> controllers, boolean[] removed) {
        for (int index = 0; index < controllers.size(); index++) {
            if (!removed[index]
                    && controllers.get(index).state() != NavigationState.COMPLETED) {
                return false;
            }
        }
        return true;
    }

    private static void runShapedSpatialCase(
            Env env, Instance instance, AsyncEntityPathfindingService service,
            Pos start, Pos goal, double width, double height, double depth,
            List<Pos> solids, long seed, String fixture) {
        TrackedCreature mob = new TrackedCreature(EntityType.BEE);
        mob.setBoundingBox(width, height, depth);
        String label = label(seed, fixture);
        try {
            mob.setInstance(instance, start).join();
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.18);
            mob.controller = controller;
            controller.moveTo(goal);
            runUntilTerminal(env, controller, 2_000);
            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> label + ", final=" + mob.getPosition()
                            + ", nodes=" + controller.nodes());
            assertCollisionFree(mob, solids, label);
            assertNoLongFreeze(mob.positions, 180, label);
        } finally {
            mob.remove();
        }
    }

    private static List<org.junit.jupiter.api.function.Executable> cases(
            String family, CaseBody body) {
        List<org.junit.jupiter.api.function.Executable> result = new ArrayList<>();
        for (int index = 0; index < CASES_PER_FAMILY; index++) {
            int caseIndex = index;
            result.add(() -> body.run(caseIndex));
        }
        return result;
    }

    private static void runCase(
            Env env, Instance instance, AsyncEntityPathfindingService service,
            EntityType type, Pos start, Pos goal, List<Pos> solids,
            NavigationMode expectedMode, long seed, String fixture) {
        TrackedCreature mob = new TrackedCreature(type);
        String label = "seed=" + Long.toUnsignedString(seed)
                + " (0x" + Long.toUnsignedString(seed, 16) + "), " + fixture;
        try {
            mob.setInstance(instance, start).join();
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.18);
            mob.controller = controller;
            assertEquals(expectedMode, BuiltinNavigationProfiles
                    .forEntityType(type).mode(), label);
            controller.moveTo(goal);
            runUntilTerminal(env, controller, 2_000);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> label + ", final=" + mob.getPosition()
                            + ", nodeIndex=" + controller.nodeIndex()
                            + ", nodes=" + controller.nodes());
            assertTrue(maximumDisplacement(mob.positions) > 2,
                    () -> label + ", follower made insufficient progress");
            assertNoLongFreeze(mob.positions, 180, label);
            assertCollisionFree(mob, solids, label);
            assertRouteWasNonTrivial(controller.nodes(), label);
        } finally {
            mob.remove();
        }
    }

    private static void runMutationCase(
            Env env, Instance instance, AsyncEntityPathfindingService service,
            EntityType type, Pos start, Pos goal, int wallX, int halfWall,
            int mutationTick, long seed, String fixture) throws Exception {
        TrackedCreature mob = new TrackedCreature(type);
        String label = "seed=" + Long.toUnsignedString(seed)
                + " (0x" + Long.toUnsignedString(seed, 16) + "), " + fixture;
        List<Pos> solids = new ArrayList<>();
        try {
            mob.setInstance(instance, start).join();
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.18);
            mob.controller = controller;
            controller.moveTo(goal);
            for (int tick = 0; tick < 300
                    && controller.state() == NavigationState.COMPUTING; tick++) {
                env.tick();
                Thread.sleep(1);
            }
            assertFalse(controller.nodes().isEmpty(),
                    () -> label + ", initial route was never accepted");
            for (int tick = 0; tick < mutationTick; tick++) env.tick();

            int minimumClearX = (int) Math.ceil(mob.getPosition().x()
                    + mob.getBoundingBox().width() * 0.5 + 1.0e-4);
            int placedWallX = Math.max(wallX, minimumClearX);
            String placedLabel = label + ", placedWallX=" + placedWallX;
            for (int y = 40; y <= 44; y++) {
                for (int z = -halfWall; z <= halfWall; z++) {
                    solid(instance, solids, placedWallX, y, z);
                }
            }
            controller.onBlockChanged(new Pos(placedWallX, 40, 0));
            runUntilTerminal(env, controller, 2_000);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> placedLabel + ", final=" + mob.getPosition()
                            + ", nodes=" + controller.nodes());
            assertCollisionFree(mob, solids, placedLabel);
            assertNoLongFreeze(mob.positions, 180, placedLabel);
            assertRouteWasNonTrivial(controller.nodes(), placedLabel);
        } finally {
            mob.remove();
        }
    }

    private static void runShapedCase(
            Env env, Instance instance, AsyncEntityPathfindingService service,
            Pos start, Pos goal, double width, double height, double depth,
            List<Pos> solids, long seed, String fixture) {
        TrackedCreature mob = new TrackedCreature(EntityType.ZOMBIE);
        mob.setBoundingBox(width, height, depth);
        String label = label(seed, fixture);
        try {
            mob.setInstance(instance, start).join();
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.18);
            mob.controller = controller;
            controller.moveTo(goal);
            runUntilTerminal(env, controller, 2_000);
            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> label + ", final=" + mob.getPosition() + ", nodes=" + controller.nodes());
            assertCollisionFree(mob, solids, label);
            assertNoLongFreeze(mob.positions, 180, label);
        } finally {
            mob.remove();
        }
    }

    private static void runMultiMutationCase(
            Env env, Instance instance, AsyncEntityPathfindingService service,
            int firstTick, int secondTick, long seed, String fixture) throws Exception {
        TrackedCreature mob = new TrackedCreature(EntityType.ZOMBIE);
        List<Pos> activeSolids = new ArrayList<>();
        String label = label(seed, fixture);
        try {
            mob.setInstance(instance, new Pos(0.5, 40, 0.5)).join();
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(13.5, 40, 0.5));
            for (int tick = 0; tick < 2_000; tick++) {
                if (tick == firstTick) {
                    for (int z = -2; z <= 2; z++) {
                        if (z == 2) continue;
                        for (int y = 40; y <= 43; y++) solid(instance, activeSolids, 5, y, z);
                    }
                    controller.onBlockChanged(new Pos(5, 40, 0));
                }
                if (tick == secondTick) {
                    for (int y = 40; y <= 43; y++) {
                        instance.setBlock(5, y, 2, Block.STONE);
                        activeSolids.add(new Pos(5, y, 2));
                        instance.setBlock(5, y, -2, Block.AIR);
                    }
                    controller.onBlockChanged(new Pos(5, 40, 2));
                    controller.onBlockChanged(new Pos(5, 40, -2));
                }
                env.tick();
                if (controller.state() == NavigationState.COMPUTING) Thread.sleep(1);
                if (controller.state() == NavigationState.COMPLETED
                        || controller.state() == NavigationState.STUCK
                        || controller.state() == NavigationState.FAILED) break;
            }
            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> label + ", final=" + mob.getPosition() + ", nodes=" + controller.nodes());
            assertNoLongFreeze(mob.positions, 180, label);
        } finally {
            mob.remove();
        }
    }

    private static void runStatefulSchedule(
            Env env, Instance instance, AsyncEntityPathfindingService service,
            int firstTick, int wallTick, int releaseTick, int side, int wallX,
            long seed, String fixture) throws Exception {
        TrackedCreature mob = new TrackedCreature(EntityType.ZOMBIE);
        String label = label(seed, fixture);
        try {
            mob.setInstance(instance, new Pos(0.5, 40, 0.5)).join();
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.2);
            mob.controller = controller;
            controller.moveTo(new Pos(13.5, 40, 0.5));
            for (int tick = 0; tick < 2_500; tick++) {
                if (tick == firstTick) {
                    controller.moveTo(new Pos(11.5, 40, side + 0.5));
                }
                if (tick == wallTick) {
                    for (int z = -5; z <= 5; z++) {
                        if (z == -side) continue;
                        for (int y = 40; y <= 43; y++) {
                            instance.setBlock(wallX, y, z, Block.STONE);
                        }
                    }
                    controller.onBlockChanged(new Pos(wallX, 40, 0));
                    controller.moveTo(new Pos(13.5, 40, -side + 0.5));
                }
                if (tick == releaseTick) {
                    for (int y = 40; y <= 43; y++) {
                        for (int z = -1; z <= 1; z++) {
                            instance.setBlock(wallX, y, z, Block.AIR);
                        }
                    }
                    controller.onBlockChanged(new Pos(wallX, 40, 0));
                    controller.moveTo(new Pos(13.5, 40, 0.5));
                }
                env.tick();
                if (controller.state() == NavigationState.COMPUTING) Thread.sleep(1);
                if (tick > releaseTick && (controller.state() == NavigationState.COMPLETED
                        || controller.state() == NavigationState.STUCK
                        || controller.state() == NavigationState.FAILED)) break;
            }
            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> label + ", final=" + mob.getPosition()
                            + ", nodeIndex=" + controller.nodeIndex()
                            + ", nodes=" + controller.nodes());
            assertNoLongFreeze(mob.positions, 240, label);
        } finally {
            mob.remove();
        }
    }

    private static String label(long seed, String fixture) {
        return "seed=" + Long.toUnsignedString(seed) + " (0x"
                + Long.toUnsignedString(seed, 16) + "), " + fixture;
    }

    private static String compactBlocks(List<Pos> solids) {
        StringBuilder result = new StringBuilder();
        for (Pos block : solids) {
            if (block.blockY() != 40) continue;
            if (!result.isEmpty()) result.append(';');
            result.append(block.blockX()).append(',').append(block.blockZ());
        }
        return result.toString();
    }

    private static void assertRouteWasNonTrivial(
            List<PathNode> nodes, String label) {
        assertTrue(nodes.size() > 3, () -> label + ", trivial route=" + nodes);
        PathNode first = nodes.getFirst();
        assertTrue(nodes.stream().anyMatch(node ->
                        node.graphY() != first.graphY()
                                || node.graphZ() != first.graphZ()
                                || node.movement() != PathNode.Movement.WALK),
                () -> label + ", route never exercised its fixture: " + nodes);
    }

    private static void assertCollisionFree(
            TrackedCreature mob, List<Pos> solids, String label) {
        for (Pos position : mob.positions) {
            for (Pos block : solids) {
                assertFalse(Block.STONE.collisionShape().intersectBox(
                                position.sub(block), mob.getBoundingBox()),
                        () -> label + ", body clipped block=" + block
                                + " at position=" + position);
            }
        }
    }

    private static void assertNoLongFreeze(
            List<Pos> positions, int limit, String label) {
        int run = 0;
        int longest = 0;
        for (int index = 1; index < positions.size(); index++) {
            if (positions.get(index).distance(positions.get(index - 1)) < 1.0e-7) {
                longest = Math.max(longest, ++run);
            } else {
                run = 0;
            }
        }
        int observed = longest;
        assertTrue(observed < limit,
                () -> label + ", froze for " + observed + " ticks");
    }

    private static double maximumDisplacement(List<Pos> positions) {
        Pos first = positions.getFirst();
        return positions.stream().mapToDouble(first::distance).max().orElse(0);
    }

    private static void runUntilTerminal(
            Env env, EntityNavigationController controller, int maximumTicks) {
        assertTimeoutPreemptively(Duration.ofSeconds(15), () -> {
            for (int tick = 0; tick < maximumTicks; tick++) {
                env.tick();
                if (controller.state() == NavigationState.COMPLETED
                        || controller.state() == NavigationState.STUCK
                        || controller.state() == NavigationState.FAILED
                        || controller.state() == NavigationState.CANCELLED) return;
                if (controller.state() == NavigationState.COMPUTING) {
                    Thread.sleep(1);
                } else {
                    Thread.yield();
                }
            }
        });
    }

    private static void reset(Instance instance) {
        fill(instance, -6, 15, 40, 56, -10, 10, Block.AIR);
        fill(instance, -6, 15, 35, 39, -10, 10, Block.STONE);
    }

    private static void fill(Instance instance,
                             int minX, int maxX, int minY, int maxY,
                             int minZ, int maxZ, Block block) {
        for (int x = minX; x <= maxX; x++) {
            for (int y = minY; y <= maxY; y++) {
                for (int z = minZ; z <= maxZ; z++) {
                    instance.setBlock(x, y, z, block);
                }
            }
        }
    }

    private static void solid(
            Instance instance, List<Pos> solids, int x, int y, int z) {
        instance.setBlock(x, y, z, Block.STONE);
        solids.add(new Pos(x, y, z));
    }

    private static <T> T pick(SplittableRandom random, T[] values) {
        return values[random.nextInt(values.length)];
    }

    private static long seed(int family, int index) {
        return new SplittableRandom(ROOT_SEED ^ ((long) family << 32) ^ index)
                .nextLong();
    }

    private static Instance prepared(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 3,
                (x, z) -> instance.loadChunk(x, z).join());
        return instance;
    }

    @FunctionalInterface
    private interface CaseBody {
        void run(int index) throws Exception;
    }

    private static final class TrackedCreature extends EntityCreature {
        private final List<Pos> positions = new ArrayList<>();
        private EntityNavigationController controller;

        private TrackedCreature(EntityType type) {
            super(type);
        }

        @Override
        public void update(long time) {
            positions.add(getPosition());
            if (controller != null) controller.tick();
            super.update(time);
        }
    }
}
