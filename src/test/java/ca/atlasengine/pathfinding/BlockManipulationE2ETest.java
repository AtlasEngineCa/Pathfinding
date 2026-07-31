package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.NavigationModifiers;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.internal.movement.BlockManipulator;
import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.OpenableBlockFamily;
import net.minestom.server.collision.BoundingBox;
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

import java.time.Duration;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Live open-on-approach and close-behind cycles for mobs that change block
 * states, checked against the resulting block states rather than the route.
 */
@EnvTest
class BlockManipulationE2ETest {
    private static final int SURFACE = 40;
    private static final BoundingBox ZOMBIE_BOX =
            new BoundingBox(0.6, 1.95, 0.6);
    private static final Block CLOSED_GATE = Block.OAK_FENCE_GATE
            .withProperty("facing", "north")
            .withProperty("open", "false");
    private static final Block CLOSED_TRAPDOOR = Block.OAK_TRAPDOOR
            .withProperty("facing", "east")
            .withProperty("half", "bottom")
            .withProperty("open", "false");
    private static final Block CLOSED_IRON_TRAPDOOR = Block.IRON_TRAPDOOR
            .withProperty("facing", "east")
            .withProperty("half", "bottom")
            .withProperty("open", "false");

    @Test
    void requestScopedCapabilityOpensAndCrossesAFenceGate(Env env) {
        Instance instance = corridor(env);
        instance.setBlock(2, SURFACE, 0, CLOSED_GATE);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature zombie = spawn(instance, new Pos(0.5, SURFACE, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(zombie, service, 0.18);
            zombie.controller = controller;
            controller.moveTo(new Pos(5.5, SURFACE, 0.5),
                    NavigationModifiers.builder()
                            .blockManipulation(BlockManipulationCapabilities.of(
                                    OpenableBlockFamily.FENCE_GATE))
                            .build());

            runUntilFinished(env, controller, 600);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + zombie.getPosition()
                            + ", nodes=" + controller.nodes());
            assertEquals("true", open(instance, 2, SURFACE));
            assertTrue(controller.nodes().stream().allMatch(
                    node -> node.graphZ() == 0));
            assertTrue(zombie.getPosition().x() > 3.5,
                    () -> "never crossed the gate: " + zombie.getPosition());
        }
    }

    @Test
    void ordinaryProfileNeitherPlansThroughNorOpensTheSameGate(Env env) {
        Instance instance = corridor(env);
        instance.setBlock(2, SURFACE, 0, CLOSED_GATE);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature zombie = spawn(instance, new Pos(0.5, SURFACE, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(zombie, service, 0.18);
            zombie.controller = controller;
            controller.moveTo(new Pos(5.5, SURFACE, 0.5));

            runUntilFinished(env, controller, 200);

            assertNotEquals(NavigationState.COMPLETED, controller.state());
            assertEquals("false", open(instance, 2, SURFACE));
            assertTrue(zombie.getPosition().x() < 2.5,
                    () -> "walked through a closed gate: "
                            + zombie.getPosition());
        }
    }

    @Test
    void closeBehindRestoresTheDoorOnceTheMobHasPassedIt(Env env) {
        Instance instance = corridor(env);
        installDoor(instance, 2, Block.OAK_DOOR);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature zombie = spawn(instance, new Pos(0.5, SURFACE, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(zombie, service,
                            zombieProfile(BlockManipulationCapabilities.STANDARD
                                    .closingBehind()), 0.18);
            zombie.controller = controller;
            controller.moveTo(new Pos(6.5, SURFACE, 0.5));

            runUntilFinished(env, controller, 600);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + zombie.getPosition()
                            + ", nodes=" + controller.nodes());
            assertTrue(zombie.getPosition().x() > 4.5,
                    () -> "never crossed the door: " + zombie.getPosition());
            assertEquals("false", open(instance, 2, SURFACE));
            assertEquals("false", open(instance, 2, SURFACE + 1));
        }
    }

    @Test
    void closeBehindIsSkippedBeyondTheConfiguredRange(Env env) {
        Instance instance = corridor(env);
        installDoor(instance, 2, Block.OAK_DOOR);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature zombie = spawn(instance, new Pos(0.5, SURFACE, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(zombie, service,
                            zombieProfile(BlockManipulationCapabilities.STANDARD
                                    .closingBehind(0.5)), 0.18);
            zombie.controller = controller;
            controller.moveTo(new Pos(6.5, SURFACE, 0.5));

            runUntilFinished(env, controller, 600);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + zombie.getPosition()
                            + ", nodes=" + controller.nodes());
            assertTrue(zombie.getPosition().x() > 4.5,
                    () -> "never crossed the door: " + zombie.getPosition());
            assertEquals("true", open(instance, 2, SURFACE),
                    "a door left out of range is forgotten, not closed");
        }
    }

    /**
     * A planned climb hangs the mob beside the ladder, so the column it opens
     * on the way up is a neighbouring one that may hold a hatch. Opening that
     * hatch swaps a floor slab for a wall panel, and the follower refuses to
     * drive into either. The route has to be planned against the states the
     * follower will actually be looking at.
     */
    @Test
    void climbsPastTheHatchItOpensBesideTheLadder(Env env) {
        Instance instance = preparedFlat(env);
        Block ladder = Block.LADDER.withProperty("facing", "north");
        for (int y = SURFACE; y <= SURFACE + 7; y++) {
            instance.setBlock(0, y, 0, ladder);
            instance.setBlock(0, y, 1, Block.STONE);
        }
        instance.setBlock(0, SURFACE + 4, -1,
                CLOSED_TRAPDOOR.withProperty("facing", "north"));

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature zombie = spawn(instance, new Pos(0.5, SURFACE, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(zombie, service,
                            climbing(zombieProfile(
                                    BlockManipulationCapabilities.of(
                                            OpenableBlockFamily.TRAPDOOR))),
                            0.18);
            zombie.controller = controller;
            controller.moveTo(new Pos(0.5, SURFACE + 6, 0.5));

            runUntilFinished(env, controller, 600);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + zombie.getPosition()
                            + ", nodes=" + controller.nodes());
            assertTrue(zombie.getPosition().y() >= SURFACE + 5,
                    () -> "never climbed past the hatch: "
                            + zombie.getPosition());
            assertEquals("true", open(instance, 0, SURFACE + 4, -1));
        }
    }

    @Test
    void eachFamilyIsIndependentAndIronVariantsAreNeverTouched(Env env) {
        Instance instance = preparedFlat(env);
        installOpenables(instance, 0);
        installOpenables(instance, 4);
        installOpenables(instance, 8);

        sweep(instance, 0, BlockManipulationCapabilities.STANDARD);
        sweep(instance, 4, BlockManipulationCapabilities.of(
                OpenableBlockFamily.TRAPDOOR));
        sweep(instance, 8, BlockManipulationCapabilities.of(
                OpenableBlockFamily.values()));

        assertEquals("true", open(instance, 0, SURFACE, 0));
        assertEquals("false", open(instance, 1, SURFACE, 0));
        assertEquals("false", open(instance, 2, SURFACE, 0));

        assertEquals("false", open(instance, 0, SURFACE, 4));
        assertEquals("true", open(instance, 1, SURFACE, 4));
        assertEquals("false", open(instance, 2, SURFACE, 4));

        assertEquals("true", open(instance, 0, SURFACE, 8));
        assertEquals("true", open(instance, 1, SURFACE, 8));
        assertEquals("true", open(instance, 2, SURFACE, 8));

        for (int z : new int[]{0, 4, 8}) {
            assertEquals("false", open(instance, 3, SURFACE, z),
                    "iron doors are never manipulated");
            assertEquals("false", open(instance, 4, SURFACE, z),
                    "iron trapdoors are never manipulated");
        }
    }

    private static void sweep(Instance instance, int z,
                              BlockManipulationCapabilities capabilities) {
        BlockManipulator manipulator = new BlockManipulator();
        MobTraversalProfile profile = MobTraversalProfile.builder("sweeper")
                .blockManipulation(capabilities)
                .build();
        for (int x = 0; x <= 4; x++) {
            manipulator.openAt(instance, profile,
                    new Vec(x + 0.5, SURFACE, z + 0.5), ZOMBIE_BOX);
        }
    }

    private static void installOpenables(Instance instance, int z) {
        installDoor(instance, 0, z, Block.OAK_DOOR);
        instance.setBlock(1, SURFACE, z, CLOSED_TRAPDOOR);
        instance.setBlock(2, SURFACE, z, CLOSED_GATE);
        installDoor(instance, 3, z, Block.IRON_DOOR);
        instance.setBlock(4, SURFACE, z, CLOSED_IRON_TRAPDOOR);
    }

    private static void installDoor(Instance instance, int x, Block door) {
        installDoor(instance, x, 0, door);
    }

    private static void installDoor(Instance instance, int x, int z, Block door) {
        Block base = door.withProperty("facing", "east")
                .withProperty("hinge", "left")
                .withProperty("open", "false");
        instance.setBlock(x, SURFACE, z, base.withProperty("half", "lower"));
        instance.setBlock(x, SURFACE + 1, z, base.withProperty("half", "upper"));
    }

    private static NavigationProfile zombieProfile(
            BlockManipulationCapabilities capabilities) {
        NavigationProfile base = BuiltinNavigationProfiles.forEntityType(
                EntityType.ZOMBIE);
        return base.withMobProfile(MobTraversalProfile.builder("manipulator")
                .from(base.mobProfile())
                .blockManipulation(capabilities)
                .build());
    }

    private static NavigationProfile climbing(NavigationProfile profile) {
        return profile.withGroundCapabilities(profile.groundCapabilities()
                .withClimbables(ClimbableCapabilities.STANDARD));
    }

    private static String open(Instance instance, int x, int y) {
        return open(instance, x, y, 0);
    }

    private static String open(Instance instance, int x, int y, int z) {
        return instance.getBlock(x, y, z).getProperty("open");
    }

    private static Instance corridor(Env env) {
        Instance instance = preparedFlat(env);
        for (int x = -20; x <= 20; x++) {
            for (int y = SURFACE; y <= SURFACE + 2; y++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
        }
        return instance;
    }

    private static Instance preparedFlat(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 3,
                (x, z) -> instance.loadChunk(x, z).join());
        return instance;
    }

    private static ControlledCreature spawn(Instance instance, Pos position) {
        ControlledCreature creature = new ControlledCreature();
        creature.setInstance(instance, position).join();
        return creature;
    }

    private static void runUntilFinished(Env env,
                                         EntityNavigationController controller,
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

    private static final class ControlledCreature extends EntityCreature {
        private EntityNavigationController controller;

        private ControlledCreature() {
            super(EntityType.ZOMBIE);
        }

        @Override
        public void update(long time) {
            if (controller != null) controller.tick();
            super.update(time);
        }
    }
}
