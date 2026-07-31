package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.metrics.NavigationMetricsSnapshot;import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import net.minestom.server.coordinate.ChunkRange;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;
import net.minestom.testing.Env;
import net.minestom.testing.EnvTest;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * A route that terminates short must say so where that is decided.
 *
 * <p>Before this, a follower penned in with a target it could not reach sat in
 * {@code PARTIAL} until the stall watchdog relabelled it {@code STUCK} two
 * hundred ticks later, which is indistinguishable from a mob wedged on a route
 * it was given and which put an unreachable target into the numerator of
 * {@code travelStallRate()}, the rate operators are told to alarm on.</p>
 */
@EnvTest
class TerminalPartialTest {
    /** Two stall windows plus the search that opens the navigation. */
    private static final int TICKS_PAST_THE_WATCHDOG = 400;

    @Test
    void anUnreachableTargetIsReportedWhereItIsDecidedAndIsNotAStall(Env env)
            throws InterruptedException {
        Instance instance = pen(env);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(instance, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(mob, service,
                            BuiltinNavigationProfiles.forEntityType(
                                    EntityType.ZOMBIE), 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(40.5, 40, 0.5));

            settle(env, controller);

            assertEquals(NavigationState.PARTIAL, controller.state(),
                    () -> "position=" + mob.getPosition()
                            + ", nodes=" + controller.nodes());
            assertTrue(controller.targetUnreachable(),
                    "a follower the planner has given up on must say so");
            NavigationMetricsSnapshot.ControllerOutcomeCounts outcomes =
                    service.metrics().snapshot().controllerOutcomes();
            assertEquals(1, outcomes.unreachableTargets(),
                    "the terminal partial is one event, not one per tick");
            assertEquals(0, outcomes.stuck(),
                    "a target that cannot be reached is not a wedged mob");
            assertEquals(0.0, outcomes.travelStallRate(),
                    "an unreachable target must stay out of the stall rate");
        }
    }

    @Test
    void aFollowerThatIsGivenAReachableTargetReportsNothing(Env env)
            throws InterruptedException {
        Instance instance = pen(env);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(instance, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(mob, service,
                            BuiltinNavigationProfiles.forEntityType(
                                    EntityType.ZOMBIE), 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(2.5, 40, 0.5));

            for (int tick = 0; tick < 200
                    && controller.state() != NavigationState.COMPLETED; tick++) {
                env.tick();
                if (controller.state() == NavigationState.COMPUTING) {
                    Thread.sleep(1);
                }
            }

            assertEquals(NavigationState.COMPLETED, controller.state());
            assertFalse(controller.targetUnreachable());
            assertEquals(0, service.metrics().snapshot().controllerOutcomes()
                    .unreachableTargets());
        }
    }

    @Test
    void aNewTargetClearsTheVerdictTheOldOneEarned(Env env)
            throws InterruptedException {
        Instance instance = pen(env);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(instance, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(mob, service,
                            BuiltinNavigationProfiles.forEntityType(
                                    EntityType.ZOMBIE), 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(40.5, 40, 0.5));
            settle(env, controller);
            assertTrue(controller.targetUnreachable());

            controller.moveTo(new Pos(2.5, 40, 0.5));

            assertFalse(controller.targetUnreachable(),
                    "a new goal is a new question");
        }
    }

    private static void settle(Env env, EntityNavigationController controller)
            throws InterruptedException {
        for (int tick = 0; tick < TICKS_PAST_THE_WATCHDOG; tick++) {
            env.tick();
            if (controller.state() == NavigationState.COMPUTING) {
                Thread.sleep(1);
            }
        }
    }

    /** A sealed five-by-five pen on flat ground, with no way out. */
    private static Instance pen(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 2,
                (x, z) -> instance.loadChunk(x, z).join());
        for (int offset = -3; offset <= 3; offset++) {
            for (int y = 40; y <= 43; y++) {
                instance.setBlock(3, y, offset, Block.STONE);
                instance.setBlock(-3, y, offset, Block.STONE);
                instance.setBlock(offset, y, 3, Block.STONE);
                instance.setBlock(offset, y, -3, Block.STONE);
            }
        }
        return instance;
    }

    private static ControlledCreature spawn(Instance instance, Pos position) {
        ControlledCreature creature = new ControlledCreature();
        creature.setInstance(instance, position).join();
        return creature;
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
