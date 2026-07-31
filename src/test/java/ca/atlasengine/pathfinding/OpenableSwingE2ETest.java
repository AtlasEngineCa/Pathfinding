package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.NavigationModifiers;import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.OpenableBlockFamily;
import net.minestom.server.collision.BoundingBox;
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

import static org.junit.jupiter.api.Assertions.*;

/**
 * Live no-stall contract for a mob that opens what it walks into. Every
 * facing, half, height and width either reaches the target or is planned
 * around; none may end wedged against the panel the mob itself swung.
 */
@EnvTest
class OpenableSwingE2ETest {
    private static final int SURFACE = 40;
    private static final String[] FACINGS = {"north", "south", "east", "west"};

    @Test
    void noTrapdoorOrientationLeavesTheMobWedgedAgainstItsOwnPanel(Env env) {
        List<String> outcomes = new ArrayList<>();
        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            for (String facing : FACINGS) {
                for (String half : new String[]{"bottom", "top"}) {
                    for (int level : new int[]{0, 1}) {
                        Instance instance = corridor(env);
                        instance.setBlock(2, SURFACE + level, 0,
                                trapdoor(facing, half));
                        outcomes.add(facing + "/" + half + "/y" + level + " "
                                + drive(env, service, instance, 0.6,
                                OpenableBlockFamily.TRAPDOOR));
                    }
                }
            }
        }

        assertEquals(List.of(
                "north/bottom/y0 COMPLETED", "north/bottom/y1 COMPLETED",
                "north/top/y0 COMPLETED", "north/top/y1 COMPLETED",
                "south/bottom/y0 COMPLETED", "south/bottom/y1 COMPLETED",
                "south/top/y0 COMPLETED", "south/top/y1 COMPLETED",
                "east/bottom/y0 PARTIAL", "east/bottom/y1 PARTIAL",
                "east/top/y0 PARTIAL", "east/top/y1 PARTIAL",
                "west/bottom/y0 PARTIAL", "west/bottom/y1 PARTIAL",
                "west/top/y0 PARTIAL", "west/top/y1 PARTIAL"), outcomes);
        for (String outcome : outcomes) {
            assertFalse(outcome.contains("STUCK"), outcome);
        }
    }

    @Test
    void aMobTooWideForThePanelIsPlannedAroundEveryFacing(Env env) {
        List<String> outcomes = new ArrayList<>();
        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            for (String facing : FACINGS) {
                Instance instance = corridor(env);
                instance.setBlock(2, SURFACE, 0, trapdoor(facing, "bottom"));
                outcomes.add(facing + " " + drive(env, service, instance, 0.9,
                        OpenableBlockFamily.TRAPDOOR));
            }
        }

        assertEquals(List.of("north PARTIAL", "south PARTIAL",
                "east PARTIAL", "west PARTIAL"), outcomes);
    }

    @Test
    void doorsAndGatesKeepTheSameContract(Env env) {
        List<String> outcomes = new ArrayList<>();
        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            for (String facing : FACINGS) {
                Instance doors = corridor(env);
                Block door = Block.OAK_DOOR.withProperty("facing", facing)
                        .withProperty("hinge", "left")
                        .withProperty("open", "false");
                doors.setBlock(2, SURFACE, 0,
                        door.withProperty("half", "lower"));
                doors.setBlock(2, SURFACE + 1, 0,
                        door.withProperty("half", "upper"));
                outcomes.add("door " + facing + " " + drive(env, service,
                        doors, 0.6, OpenableBlockFamily.DOOR));

                Instance gates = corridor(env);
                gates.setBlock(2, SURFACE, 0,
                        Block.OAK_FENCE_GATE.withProperty("facing", facing)
                                .withProperty("open", "false"));
                outcomes.add("gate " + facing + " " + drive(env, service,
                        gates, 0.6, OpenableBlockFamily.FENCE_GATE));
            }
        }

        // A door swings along the doorway it is hung in and across any other
        // axis, so only the two facings that leave the corridor open cross.
        assertEquals(List.of(
                "door north PARTIAL", "gate north COMPLETED",
                "door south PARTIAL", "gate south COMPLETED",
                "door east COMPLETED", "gate east COMPLETED",
                "door west COMPLETED", "gate west COMPLETED"), outcomes);
        for (String outcome : outcomes) {
            assertFalse(outcome.contains("STUCK"), outcome);
        }
    }

    private String drive(Env env, AsyncEntityPathfindingService service,
                         Instance instance, double width,
                         OpenableBlockFamily family) {
        ControlledCreature mob = new ControlledCreature();
        mob.setBoundingBox(width, 1.95, width);
        mob.setInstance(instance, new Pos(0.5, SURFACE, 0.5)).join();
        EntityNavigationController controller =
                EntityNavigationController.builtin(mob, service, 0.18);
        mob.controller = controller;
        controller.moveTo(new Pos(4.5, SURFACE, 0.5),
                NavigationModifiers.builder()
                        .blockManipulation(
                                BlockManipulationCapabilities.of(family))
                        .build());
        assertTimeoutPreemptively(Duration.ofSeconds(20), () -> {
            for (int tick = 0; tick < 400; tick++) {
                env.tick();
                if (terminal(controller.state())) return;
                if (controller.state() == NavigationState.COMPUTING) {
                    Thread.sleep(1);
                } else {
                    Thread.yield();
                }
            }
        });
        NavigationState state = controller.state();
        Pos position = mob.getPosition();
        mob.remove();
        assertTrue(terminal(state),
                () -> "never settled: " + state + " at " + position);
        // A mob wedged in the panel it opened would be standing inside it.
        assertFalse(occupied(instance, position, mob.getBoundingBox()),
                () -> "wedged in a swung panel at " + position);
        return state.name();
    }

    private static boolean terminal(NavigationState state) {
        return state == NavigationState.COMPLETED
                || state == NavigationState.PARTIAL
                || state == NavigationState.STUCK
                || state == NavigationState.FAILED
                || state == NavigationState.CANCELLED;
    }

    private static boolean occupied(Instance instance, Pos position,
                                    BoundingBox box) {
        var iterator = box.getBlocks(position);
        while (iterator.hasNext()) {
            var mutable = iterator.next();
            Pos cell = new Pos(mutable.blockX(), mutable.blockY(),
                    mutable.blockZ());
            if (instance.getBlock(cell).collisionShape()
                    .intersectBox(position.sub(cell), box)) return true;
        }
        return false;
    }

    private static Block trapdoor(String facing, String half) {
        return Block.OAK_TRAPDOOR.withProperty("facing", facing)
                .withProperty("half", half).withProperty("open", "false");
    }

    /** Sealed one-wide corridor: a refused cell has no detour around it. */
    private static Instance corridor(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 2,
                (x, z) -> instance.loadChunk(x, z).join());
        for (int x = -1; x <= 6; x++) {
            for (int y = SURFACE; y <= SURFACE + 2; y++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
        }
        for (int y = SURFACE; y <= SURFACE + 2; y++) {
            instance.setBlock(-1, y, 0, Block.STONE);
            instance.setBlock(6, y, 0, Block.STONE);
        }
        return instance;
    }

    private static final class ControlledCreature extends EntityCreature {
        private EntityNavigationController controller;

        private ControlledCreature() {
            super(EntityType.VILLAGER);
        }

        @Override
        public void update(long time) {
            if (controller != null) controller.tick();
            super.update(time);
        }
    }
}
