package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import net.minestom.server.coordinate.ChunkRange;
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
 * The capability the terminal radius used to cost, end to end.
 *
 * <p>A follower whose search spends its whole visit budget against a barrier
 * ends five blocks from a target it can reach, because the way round is
 * further than one bounded search covers. Reading only the straight line, that
 * is inside the terminal radius and the navigation ends there. Reading why the
 * search stopped, the frontier still held cells and the segment just walked
 * was five times the distance left, so another bounded segment is warranted
 * and it arrives.</p>
 */
@EnvTest
class PartialChainRecoveryTest {
    private static final int WALL_X = 26;
    private static final int GAP_Z = 9;

    @Test
    void aBudgetBoundPartialChainsOnToAReachableTargetInsideTheRadius(Env env)
            throws InterruptedException {
        Instance instance = barrier(env);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(instance, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(mob, service,
                            BuiltinNavigationProfiles.forEntityType(
                                    EntityType.ZOMBIE), 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(30.5, 40, 0.5));

            boolean reachedWall = false;
            for (int tick = 0; tick < 1_500
                    && controller.state() != NavigationState.COMPLETED; tick++) {
                env.tick();
                if (controller.state() == NavigationState.COMPUTING) {
                    Thread.sleep(1);
                }
                if (mob.getPosition().x() > 24 && mob.getPosition().z() < 2) {
                    reachedWall = true;
                }
            }

            assertTrue(reachedWall,
                    "the fixture must first strand the follower at the wall");
            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition()
                            + ", routeStop=" + controller.routeStop()
                            + ", unreachable=" + controller.targetUnreachable()
                            + ", nodes=" + controller.nodes());
            assertFalse(controller.targetUnreachable(),
                    "a target the follower reached was never unreachable");
            assertEquals(0, service.metrics().snapshot().controllerOutcomes()
                    .unreachableTargets());
            assertTrue(service.metrics().snapshot().searches().submitted() >= 2,
                    "arriving took more than the one bounded search");
        }
    }

    /**
     * Open ground with one long barrier and a distant gap. The barrier stops
     * short of the loaded edge so nothing is sealed: the search has plenty of
     * plain left on its frontier when the visit budget ends it.
     */
    private static Instance barrier(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 3,
                (x, z) -> instance.loadChunk(x, z).join());
        for (int z = -30; z <= 30; z++) {
            if (z == GAP_Z) continue;
            for (int y = 40; y <= 42; y++) {
                instance.setBlock(WALL_X, y, z, Block.STONE);
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
