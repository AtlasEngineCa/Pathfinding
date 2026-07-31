package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.internal.movement.MovementContext;
import ca.atlasengine.pathfinding.internal.movement.PlatformJumpMovementExecutor;
import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.OpenableBlockFamily;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.ChunkRange;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;
import net.minestom.testing.Env;
import net.minestom.testing.EnvTest;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Live half of {@link PlatformJumpDoorTest}. The search reads a jump's
 * landing column as already open, so the launch is what has to open it, and
 * these fixtures check the block states the run actually leaves behind.
 */
@EnvTest
class PlatformJumpDoorE2ETest {
    private static final int SURFACE = 40;
    private static final Block CLOSED_GATE = Block.OAK_FENCE_GATE
            .withProperty("facing", "north")
            .withProperty("open", "false");

    @Test
    void aMobThatOpensGatesJumpsOntoTheGatedPlatformAndLaunchesOnce(Env env) {
        Instance instance = lane(env);
        instance.setBlock(4, SURFACE, 0, CLOSED_GATE);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(instance,
                    new Pos(0.5, SURFACE, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(mob, service,
                            openingJumper(), 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(8.5, SURFACE, 0.5));

            runUntilFinished(env, controller, 1_200);

            long planned = controller.nodes().stream().filter(node ->
                    node.movement() == PathNode.Movement.JUMP).count();
            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition() + " nodes="
                            + controller.nodes());
            assertEquals(1, planned, controller.nodes()::toString);
            assertEquals(planned, mob.upwardLaunches,
                    "every planned jump has to be launched");
            assertEquals("true",
                    instance.getBlock(4, SURFACE, 0).getProperty("open"),
                    "the landing gate has to be open by the time it is used");
            assertTrue(mob.getPosition().x() > 4.5,
                    () -> "never crossed the gap: " + mob.getPosition());
            assertTrue(mob.positions.stream().mapToDouble(Pos::y).min()
                            .orElseThrow() >= SURFACE - 0.01,
                    () -> "fell into the gap: " + mob.positions);
            assertTrajectoryIsCollisionFree(mob);
        }
    }

    /**
     * The pairing without the controller in the way. Nothing else has opened
     * the gate when the executor is asked to fly this arc, so a launch here
     * is the follower doing its own half of the rule the search plans by.
     */
    @ParameterizedTest
    @ValueSource(booleans = {true, false})
    void theLaunchOpensItsOwnLandingBeforeItSweepsTheArc(
            boolean opens, Env env) {
        Instance instance = lane(env);
        instance.setBlock(4, SURFACE, 0, CLOSED_GATE);
        EntityCreature mob = new EntityCreature(EntityType.ZOMBIE);
        mob.setInstance(instance, new Pos(1.5, SURFACE, 0.5)).join();
        RecordingContext context = new RecordingContext(mob, opens,
                new PathNode(1.5, SURFACE, 0.5, PathNode.Movement.WALK,
                        1, SURFACE, 0),
                new PathNode(4.5, SURFACE, 0.5, PathNode.Movement.JUMP,
                        4, SURFACE, 0));
        PlatformJumpMovementExecutor executor =
                new PlatformJumpMovementExecutor();

        for (int tick = 0; tick < 40 && context.startedJumps == 0
                && context.recomputes == 0; tick++) {
            executor.move(context, context.current.asVec());
            env.tick();
        }

        assertEquals(opens ? "true" : "false",
                instance.getBlock(4, SURFACE, 0).getProperty("open"),
                "only a profile that opens gates may touch this one");
        assertEquals(opens ? 1 : 0, context.startedJumps,
                () -> "opens=" + opens + " launched " + context.startedJumps);
        assertEquals(opens ? 0 : 1, context.recomputes,
                () -> "opens=" + opens + " refused " + context.recomputes
                        + " launches");
        mob.remove();
    }

    /**
     * A door landing, unlike a gate, is still a shape once it is open, and a
     * profile that closes behind itself leaves it shut. The world the run
     * ends in is therefore not the world the arc flew through, so the sweep
     * only means anything against the state each tick actually held.
     */
    @Test
    void aClosingProfileJumpsOntoADoorLandingWithoutClippingIt(Env env) {
        Instance instance = lane(env);
        Block door = Block.OAK_DOOR.withProperty("facing", "east")
                .withProperty("hinge", "left").withProperty("open", "false");
        instance.setBlock(4, SURFACE, 0, door.withProperty("half", "lower"));
        instance.setBlock(4, SURFACE + 1, 0,
                door.withProperty("half", "upper"));

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(instance,
                    new Pos(0.5, SURFACE, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(mob, service,
                            closingJumper(), 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(8.5, SURFACE, 0.5));

            runUntilFinished(env, controller, 1_200);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition() + " door="
                            + instance.getBlock(4, SURFACE, 0).properties()
                            + " nodes=" + controller.nodes());
            assertEquals(1, controller.nodes().stream().filter(node ->
                            node.movement() == PathNode.Movement.JUMP).count(),
                    controller.nodes()::toString);
            assertEquals(1, mob.upwardLaunches,
                    "the door landing has to be crossed by the planned jump");
            assertEquals("false",
                    instance.getBlock(4, SURFACE, 0).getProperty("open"),
                    "a closing profile leaves the landing shut, so the final "
                            + "world is not the world the arc flew through");
            assertEquals("false",
                    instance.getBlock(4, SURFACE + 1, 0).getProperty("open"));
            assertTrue(mob.getPosition().x() > 4.5,
                    () -> "never crossed the gap: " + mob.getPosition());
            assertTrajectoryIsCollisionFree(mob);
        }
    }

    private static NavigationProfile openingJumper() {
        return jumper(BlockManipulationCapabilities.of(
                OpenableBlockFamily.values()));
    }

    private static NavigationProfile closingJumper() {
        return jumper(BlockManipulationCapabilities.of(
                OpenableBlockFamily.values()).closingBehind());
    }

    private static NavigationProfile jumper(
            BlockManipulationCapabilities capabilities) {
        NavigationProfile base = BuiltinNavigationProfiles.forEntityType(
                EntityType.ZOMBIE);
        return base.withMobProfile(MobTraversalProfile.builder("opener")
                        .from(base.mobProfile())
                        .blockManipulation(capabilities)
                        .build())
                .withGroundCapabilities(base.groundCapabilities()
                        .withPlatformJump(
                                PlatformJumpCapabilities.acrossGaps(2)));
    }

    /** A one-cell lane walled in z, cut by a gap the mob has to jump. */
    private static Instance lane(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 3,
                (x, z) -> instance.loadChunk(x, z).join());
        for (int y = SURFACE; y <= SURFACE + 3; y++) {
            for (int x = -3; x <= 13; x++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
            for (int z = -1; z <= 1; z++) {
                instance.setBlock(-3, y, z, Block.STONE);
                instance.setBlock(13, y, z, Block.STONE);
            }
        }
        for (int x = 2; x <= 3; x++) {
            for (int y = 0; y < SURFACE; y++) {
                instance.setBlock(x, y, 0, Block.AIR);
            }
        }
        return instance;
    }

    private static ControlledCreature spawn(Instance instance, Pos position) {
        ControlledCreature creature = new ControlledCreature();
        creature.setInstance(instance, position).join();
        return creature;
    }

    private static void runUntilFinished(
            Env env, EntityNavigationController controller,
            int maximumTicks) {
        assertTimeoutPreemptively(Duration.ofSeconds(20), () -> {
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

    private static void assertTrajectoryIsCollisionFree(
            ControlledCreature creature) {
        assertNotNull(creature.getInstance());
        assertFalse(creature.sweeps.isEmpty(), "no per-tick sweep recorded");
        BoundingBox box = creature.getBoundingBox();
        for (Sweep sweep : creature.sweeps) {
            for (int i = 0; i < sweep.blocks().size(); i++) {
                Pos block = sweep.blocks().get(i);
                Block value = sweep.states().get(i);
                assertFalse(value.collisionShape()
                                .intersectBox(sweep.position().sub(block), box),
                        () -> "trajectory intersected " + value.key()
                                + " at " + block + " from " + sweep.position());
            }
        }
    }

    /** Drives one executor directly, with no controller and no manipulator. */
    private static final class RecordingContext implements MovementContext {
        private final Entity entity;
        private final boolean opens;
        private final PathNode previous;
        private final PathNode current;
        private int recomputes;
        private int startedJumps;

        private RecordingContext(Entity entity, boolean opens,
                                 PathNode previous, PathNode current) {
            this.entity = entity;
            this.opens = opens;
            this.previous = previous;
            this.current = current;
        }

        @Override
        public Entity entity() {
            return entity;
        }

        @Override
        public double movementPerTick() {
            return 0.2;
        }

        @Override
        public MovementExecutionMode executionMode() {
            return MovementExecutionMode.PHYSICS_VELOCITY;
        }

        @Override
        public NavigationProfile profile() {
            NavigationProfile profile = openingJumper();
            return opens ? profile : profile.withMobProfile(
                    MobTraversalProfile.builder("plain")
                            .from(profile.mobProfile())
                            .blockManipulation(
                                    BlockManipulationCapabilities.DISABLED)
                            .build());
        }

        @Override
        public PathNode currentNode() {
            return current;
        }

        @Override
        public PathNode previousNode() {
            return previous;
        }

        @Override
        public boolean wallClimberFallback() {
            return false;
        }

        @Override
        public boolean claimGroundImpulse() {
            return false;
        }

        @Override
        public boolean platformJumpStarted() {
            return startedJumps > 0;
        }

        @Override
        public int countPlatformJumpAlignment() {
            return 1;
        }

        @Override
        public void startPlatformJump(double horizontalSpeed) {
            startedJumps++;
        }

        @Override
        public double platformJumpSpeed() {
            return 0.2;
        }

        @Override
        public void requestRecompute() {
            recomputes++;
        }
    }

    private static final class ControlledCreature extends EntityCreature {
        private EntityNavigationController controller;
        private final List<Pos> positions = new ArrayList<>();
        private final List<Sweep> sweeps = new ArrayList<>();
        private int upwardLaunches;

        private ControlledCreature() {
            super(EntityType.ZOMBIE);
        }

        @Override
        public void update(long time) {
            Pos beforePosition = getPosition();
            if (controller != null) {
                double beforeVerticalVelocity = getVelocity().y();
                controller.tick();
                if (beforeVerticalVelocity <= 0 && getVelocity().y() >= 4) {
                    upwardLaunches++;
                }
            }
            sweeps.add(sweep(beforePosition));
            super.update(time);
            positions.add(getPosition());
            sweeps.add(sweep(getPosition()));
        }

        private Sweep sweep(Pos position) {
            Instance instance = getInstance();
            List<Pos> blocks = new ArrayList<>();
            List<Block> states = new ArrayList<>();
            var iterator = getBoundingBox().getBlocks(position);
            while (iterator.hasNext()) {
                var mutable = iterator.next();
                Pos block = new Pos(mutable.blockX(),
                        mutable.blockY(), mutable.blockZ());
                blocks.add(block);
                states.add(instance.getBlock(block));
            }
            return new Sweep(position, List.copyOf(blocks),
                    List.copyOf(states));
        }
    }

    /**
     * One position beside the states its box's blocks held on that tick. A
     * landing column changes shape mid-run, so sweeping the world the run
     * ends in would test shapes the entity never moved through.
     */
    private record Sweep(Pos position, List<Pos> blocks, List<Block> states) {
    }
}
