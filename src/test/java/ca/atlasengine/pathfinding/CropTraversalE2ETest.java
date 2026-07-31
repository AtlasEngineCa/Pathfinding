package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.TerrainClassifier;
import ca.atlasengine.pathfinding.terrain.TerrainType;
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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

/**
 * Crops must not stop a mob.
 *
 * <p>Minestom reports this twice, still open at the time of writing:
 * <a href="https://github.com/Minestom/Minestom/issues/3224">#3224</a>, where
 * mobs "do not walk through and get stuck" on wheat, carrot, potato and berry
 * bush and "if you spawn them right into it, they will not move at all", and
 * <a href="https://github.com/Minestom/Minestom/issues/2813">#2813</a>, where
 * vegetables are seen "as blocked path".</p>
 *
 * <p>A crop has no collision, so the rule that decides this is the baseline
 * one: a state is pathfindable unless its collision shape is a full cube.
 * These fixtures walk real ticked entities through a planted field so a
 * regression shows up as a mob that stops rather than as a changed constant.</p>
 */
@EnvTest
class CropTraversalE2ETest {
    private static final Block[] CROPS = {
            Block.WHEAT, Block.CARROTS, Block.POTATOES, Block.BEETROOTS};

    @Test
    void everyCropIsOpenGroundRatherThanABlockedCell() {
        for (Block crop : CROPS) {
            assertEquals(TerrainType.OPEN, TerrainClassifier.raw(crop),
                    () -> crop.key() + " must be walkable, not blocked");
            assertTrue(BlockTraversalData.isLandPathfindable(crop),
                    () -> crop.key() + " has no collision, so land movement "
                            + "passes through it");
        }
        // A berry bush hurts, so it is priced rather than closed: baseline
        // routes around it when it can and through it when it must.
        assertEquals(TerrainType.DAMAGING,
                TerrainClassifier.raw(Block.SWEET_BERRY_BUSH));
        assertTrue(BlockTraversalData.isLandPathfindable(Block.SWEET_BERRY_BUSH),
                "a berry bush is costly, not impassable");
    }

    @Test
    void aMobWalksThroughAPlantedFieldInsteadOfStopping(Env env) {
        Instance instance = plantedField(env, Block.WHEAT);
        EntityCreature mob = new EntityCreature(EntityType.ZOMBIE);
        mob.setInstance(instance, new Pos(0.5, 41, 0.5)).join();
        env.tick();

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            EntityNavigationController controller = navigation.controller(mob);
            controller.moveTo(new Pos(11.5, 41, 0.5));
            walkUntilSettled(controller, env);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "a mob must cross a wheat field; ended "
                            + controller.state() + " at " + mob.getPosition());
            assertTrue(mob.getPosition().x() > 10,
                    () -> "the mob stopped inside the field at "
                            + mob.getPosition());
        } finally {
            mob.remove();
        }
    }

    @Test
    void aMobSpawnedInsideCropsStillMoves(Env env) {
        // The exact report in #3224: spawned into the crop, never moves.
        Instance instance = plantedField(env, Block.CARROTS);
        EntityCreature mob = new EntityCreature(EntityType.ZOMBIE);
        mob.setInstance(instance, new Pos(4.5, 41, 0.5)).join();
        env.tick();

        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(1).queueCapacity(8).build()) {
            EntityNavigationController controller = navigation.controller(mob);
            controller.moveTo(new Pos(11.5, 41, 0.5));
            walkUntilSettled(controller, env);

            assertNotEquals(NavigationState.STUCK, controller.state(),
                    "a mob standing in crops is not wedged");
            assertTrue(mob.getPosition().x() > 5.5,
                    () -> "the mob never left the crop it spawned in: "
                            + mob.getPosition());
        } finally {
            mob.remove();
        }
    }

    @Test
    void aRouteIsFoundThroughEveryCropFamily(Env env) {
        for (Block crop : CROPS) {
            Instance instance = plantedField(env, crop);
            EntityCreature mob = new EntityCreature(EntityType.ZOMBIE);
            mob.setInstance(instance, new Pos(0.5, 41, 0.5)).join();
            env.tick();
            try (NavigationSystem navigation = NavigationSystem.builder()
                    .parallelism(1).queueCapacity(8).build()) {
                EntityNavigationController controller =
                        navigation.controller(mob);
                controller.moveTo(new Pos(11.5, 41, 0.5));
                long deadline =
                        System.nanoTime() + Duration.ofSeconds(10).toNanos();
                while (controller.nodes().isEmpty()) {
                    if (System.nanoTime() > deadline) {
                        fail(crop.key() + " produced no route at all");
                    }
                    controller.tick();
                    env.tick();
                }
                boolean crossesField = controller.nodes().stream()
                        .anyMatch(node -> node.x() > 4 && node.x() < 8);
                assertTrue(crossesField,
                        () -> "the route around " + crop.key()
                                + " avoided the field instead of crossing it");
                controller.close();
            } finally {
                mob.remove();
            }
        }
    }

    /** Soil with a crop planted on top, the shape the reports show. */
    private static Instance plantedField(Env env, Block crop) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 2,
                (x, z) -> instance.loadChunk(x, z).join());
        for (int x = -2; x <= 14; x++) {
            for (int z = -3; z <= 3; z++) {
                instance.setBlock(x, 40, z, Block.DIRT);
                instance.setBlock(x, 41, z, Block.AIR);
            }
        }
        // The planted strip the mob has to cross.
        for (int x = 3; x <= 9; x++) {
            for (int z = -2; z <= 2; z++) {
                instance.setBlock(x, 40, z, Block.FARMLAND);
                instance.setBlock(x, 41, z, crop);
            }
        }
        return instance;
    }

    private static void walkUntilSettled(
            EntityNavigationController controller, Env env) {
        long deadline = System.nanoTime() + Duration.ofSeconds(20).toNanos();
        while (controller.state() != NavigationState.COMPLETED
                && controller.state() != NavigationState.STUCK
                && controller.state() != NavigationState.FAILED) {
            if (System.nanoTime() > deadline) return;
            controller.tick();
            env.tick();
        }
    }
}
