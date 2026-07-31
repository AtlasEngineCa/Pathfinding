package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.result.PathNode;import net.minestom.server.coordinate.ChunkRange;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;
import net.minestom.testing.Env;
import net.minestom.testing.EnvTest;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Live proof that the follower picks its step path by measured rise. Every
 * fixture sits in a sealed one-lane corridor, so a run that reports progress
 * past it stood on or in that cell and a refusal cannot be a detour.
 *
 * <p>The default grid runs the baseline 0.6 width on all four approaches.
 * {@code -Dpathfinding.stepGridScenarios=all} adds the two-cell 1.4 width.</p>
 */
@EnvTest
class ShapedStepFollowTest {
    private static final String GRID_PROPERTY = "pathfinding.stepGridScenarios";
    private static final int SURFACE = 40;
    private static final double[] DEFAULT_WIDTHS = {0.6};
    private static final double[] ALL_WIDTHS = {0.6, 1.4};

    /**
     * A travel direction and the perpendicular the corridor is laid out on.
     * Sweeping all four turns one stair state into all four of its
     * facing-relative approaches.
     */
    private enum Approach {
        EAST(1, 0, 0, 1), WEST(-1, 0, 0, 1),
        SOUTH(0, 1, 1, 0), NORTH(0, -1, 1, 0);

        private final int ax;
        private final int az;
        private final int px;
        private final int pz;

        Approach(int ax, int az, int px, int pz) {
            this.ax = ax;
            this.az = az;
            this.px = px;
            this.pz = pz;
        }

        private int x(int along, int across) {
            return along * ax + across * px;
        }

        private int z(int along, int across) {
            return along * az + across * pz;
        }

        private Pos at(double along, double across, double y) {
            return new Pos(along * ax + across * px, y,
                    along * az + across * pz);
        }

        private double travelled(Pos position) {
            return position.x() * ax + position.z() * az;
        }
    }

    private record Run(NavigationState state, Pos end, double progress,
                       double peak, int launches, List<PathNode> nodes) {
    }

    @Test
    void blocksWithOneCollisionShapeFollowOneStepPath(Env env) {
        Block stone = Block.STONE;
        Block doubleSlab = Block.STONE_SLAB.withProperty("type", "double");
        assertEquals(stone.collisionShape().relativeStart(),
                doubleSlab.collisionShape().relativeStart());
        assertEquals(stone.collisionShape().relativeEnd(),
                doubleSlab.collisionShape().relativeEnd());

        Run walked = cross(env, stone, 0.6, Approach.EAST);
        Run slabbed = cross(env, doubleSlab, 0.6, Approach.EAST);

        assertEquals(NavigationState.COMPLETED, walked.state(),
                () -> "stone: " + walked);
        assertEquals(walked.state(), slabbed.state(),
                () -> "shape-equal blocks diverged: stone=" + walked
                        + ", double slab=" + slabbed);
        assertEquals(walked.nodes(), slabbed.nodes(),
                "shape-equal blocks were planned differently");
        assertEquals(walked.launches(), slabbed.launches(),
                () -> "shape-equal blocks used different step paths: stone="
                        + walked + ", double slab=" + slabbed);
        assertEquals(walked.end().x(), slabbed.end().x(), 1.0e-9,
                () -> "shape-equal blocks ended apart: stone=" + walked
                        + ", double slab=" + slabbed);
        assertEquals(walked.end().y(), slabbed.end().y(), 1.0e-9,
                () -> "shape-equal blocks ended apart: stone=" + walked
                        + ", double slab=" + slabbed);
    }

    @Test
    void everyShapedStateIsCrossedByMeasuredRise(Env env) {
        List<String> stalled = new ArrayList<>();
        int crossed = 0;
        for (double width : widths()) {
            int lane = lane(width);
            for (Approach approach : Approach.values()) {
                for (Map.Entry<String, Block> entry : fixtures().entrySet()) {
                    Run run = cross(env, entry.getValue(), width, approach);
                    if (run.state() == NavigationState.COMPLETED
                            && run.progress() > 3 + lane) {
                        crossed++;
                        continue;
                    }
                    stalled.add(String.format(Locale.ROOT,
                            "w=%.1f %s %s -> %s", width, approach,
                            entry.getKey(), run));
                }
            }
        }
        int total = crossed + stalled.size();
        System.out.printf(Locale.ROOT,
                "SHAPED-STEP grid crossed=%d/%d%n", crossed, total);
        assertEquals(widths().length * Approach.values().length
                * fixtures().size(), total);
        assertTrue(stalled.isEmpty(),
                () -> "shaped fixtures the follower could not cross: "
                        + stalled);
    }

    @Test
    void snowLayersCrossOnEitherSideOfTheStepHeight(Env env) {
        // Layer 5 tops out at 0.5 and layer 6 at 0.625, which brackets the
        // baseline step height. Both must cross; only the taller one may need
        // an impulse.
        Run stepped = cross(env, Block.SNOW.withProperty("layers", "5"),
                0.6, Approach.EAST);
        Run jumped = cross(env, Block.SNOW.withProperty("layers", "6"),
                0.6, Approach.EAST);

        assertEquals(NavigationState.COMPLETED, stepped.state(),
                () -> "snow 5: " + stepped);
        assertEquals(NavigationState.COMPLETED, jumped.state(),
                () -> "snow 6: " + jumped);
        assertEquals(0, stepped.launches(),
                () -> "a 0.5 rise must not launch: " + stepped);
        assertEquals(1, jumped.launches(),
                () -> "a rise past the step height must launch once: "
                        + jumped);
    }

    @Test
    void staircasesAreClimbedTreadByTread(Env env) {
        Map<String, Block> treads = new LinkedHashMap<>();
        treads.put("stone", Block.STONE);
        treads.put("double_slab",
                Block.STONE_SLAB.withProperty("type", "double"));
        treads.put("stairs_with_us", stair("east"));
        treads.put("stairs_against_us", stair("west"));
        for (Map.Entry<String, Block> entry : treads.entrySet()) {
            Run run = climbFlight(env, entry.getValue(), 6);
            assertEquals(NavigationState.COMPLETED, run.state(),
                    () -> entry.getKey() + " flight: " + run);
            assertTrue(run.peak() >= SURFACE + 6 - 0.1,
                    () -> entry.getKey()
                            + " flight was not climbed to the landing: "
                            + run);
            System.out.printf(Locale.ROOT,
                    "SHAPED-STEP flight %s peak=%.3f launches=%d%n",
                    entry.getKey(), run.peak(), run.launches());
        }
    }

    private static Block stair(String facing) {
        return Block.OAK_STAIRS.withProperty("facing", facing)
                .withProperty("half", "bottom")
                .withProperty("shape", "straight");
    }

    /**
     * Walks a 0.6-wide mob up a flight of {@code treads} treads to a walled
     * landing. Every tread is the same block, so a run that reaches the top
     * chained the climb rather than managing a single tread.
     */
    private static Run climbFlight(Env env, Block tread, int treads) {
        Instance instance = preparedFlat(env);
        for (int step = 0; step < treads; step++) {
            int x = 2 + step;
            for (int y = SURFACE; y < SURFACE + step; y++) {
                instance.setBlock(x, y, 0, Block.STONE);
            }
            instance.setBlock(x, SURFACE + step, 0, tread);
        }
        for (int x = 2 + treads; x <= 2 + treads + 4; x++) {
            for (int y = SURFACE; y < SURFACE + treads; y++) {
                instance.setBlock(x, y, 0, Block.STONE);
            }
        }
        // Keep the mob on the flight: with walls on both sides the only route
        // to the landing climbs every tread.
        for (int x = -1; x <= 2 + treads + 5; x++) {
            for (int y = SURFACE; y <= SURFACE + treads + 4; y++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
        }
        return follow(env, instance, 0.6, new Pos(0.5, SURFACE, 0.5),
                new Pos(2 + treads + 3.5, SURFACE + treads, 0.5),
                Approach.EAST, 2_000);
    }

    /**
     * Sends a mob down a sealed corridor whose only obstacle is one fixture
     * tread laid across the lane.
     */
    private static Run cross(Env env, Block fixture, double width,
                             Approach approach) {
        int lane = lane(width);
        Instance instance = preparedFlat(env);
        for (int along = -3; along <= 12; along++) {
            for (int across : new int[]{-1, lane}) {
                for (int y = SURFACE; y <= SURFACE + 5; y++) {
                    instance.setBlock(approach.x(along, across), y,
                            approach.z(along, across), Block.STONE);
                }
            }
        }
        for (int along : new int[]{-2, 10}) {
            for (int across = 0; across < lane; across++) {
                for (int y = SURFACE; y <= SURFACE + 5; y++) {
                    instance.setBlock(approach.x(along, across), y,
                            approach.z(along, across), Block.STONE);
                }
            }
        }
        for (int along = 3; along < 3 + lane; along++) {
            for (int across = 0; across < lane; across++) {
                instance.setBlock(approach.x(along, across), SURFACE,
                        approach.z(along, across), fixture);
            }
        }
        double centre = lane == 1 ? 0.5 : 1.0;
        return follow(env, instance, width,
                approach.at(centre, centre, SURFACE),
                approach.at(6 + centre, centre, SURFACE), approach, 700);
    }

    private static Run follow(Env env, Instance instance, double width,
                              Pos start, Pos goal, Approach approach,
                              int maximumTicks) {
        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            TrackedCreature mob = new TrackedCreature(EntityType.ZOMBIE);
            mob.setBoundingBox(width, 1.95, width);
            mob.setInstance(instance, start).join();
            EntityNavigationController controller =
                    EntityNavigationController.builtin(mob, service, 0.12);
            mob.controller = controller;
            controller.moveTo(goal);
            for (int tick = 0; tick < maximumTicks; tick++) {
                env.tick();
                NavigationState state = controller.state();
                if (state == NavigationState.COMPLETED
                        || state == NavigationState.STUCK
                        || state == NavigationState.FAILED
                        || state == NavigationState.CANCELLED) {
                    break;
                }
                if (state == NavigationState.COMPUTING) {
                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException interrupted) {
                        Thread.currentThread().interrupt();
                    }
                } else {
                    Thread.yield();
                }
            }
            double progress = mob.positions.stream()
                    .mapToDouble(approach::travelled).max().orElse(0);
            double peak = mob.positions.stream()
                    .mapToDouble(Pos::y).max().orElse(0);
            return new Run(controller.state(), mob.getPosition(), progress,
                    peak, mob.upwardLaunches, controller.nodes());
        }
    }

    private static Map<String, Block> fixtures() {
        Map<String, Block> fixtures = new LinkedHashMap<>();
        fixtures.put("stone", Block.STONE);
        for (String type : new String[]{"top", "bottom", "double"}) {
            fixtures.put("stone_slab[" + type + "]",
                    Block.STONE_SLAB.withProperty("type", type));
        }
        for (int layers = 1; layers <= 8; layers++) {
            fixtures.put("snow[" + layers + "]",
                    Block.SNOW.withProperty("layers", String.valueOf(layers)));
        }
        fixtures.put("white_carpet", Block.WHITE_CARPET);
        for (String facing : new String[]{"north", "south", "east", "west"}) {
            for (String half : new String[]{"bottom", "top"}) {
                for (String shape : new String[]{"straight", "inner_left",
                        "inner_right", "outer_left", "outer_right"}) {
                    fixtures.put("oak_stairs[" + facing + "," + half + ","
                                    + shape + "]",
                            Block.OAK_STAIRS.withProperty("facing", facing)
                                    .withProperty("half", half)
                                    .withProperty("shape", shape));
                }
            }
        }
        return fixtures;
    }

    private static double[] widths() {
        return "all".equals(System.getProperty(GRID_PROPERTY))
                ? ALL_WIDTHS : DEFAULT_WIDTHS;
    }

    private static int lane(double width) {
        return (int) Math.ceil(width);
    }

    private static Instance preparedFlat(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 3,
                (x, z) -> instance.loadChunk(x, z).join());
        return instance;
    }

    private static final class TrackedCreature extends EntityCreature {
        private EntityNavigationController controller;
        private final List<Pos> positions = new ArrayList<>();
        private int upwardLaunches;

        private TrackedCreature(EntityType entityType) {
            super(entityType);
        }

        @Override
        public void update(long time) {
            if (controller != null) {
                double beforeVerticalVelocity = getVelocity().y();
                controller.tick();
                if (beforeVerticalVelocity <= 0 && getVelocity().y() >= 4) {
                    upwardLaunches++;
                }
            }
            super.update(time);
            positions.add(getPosition());
        }
    }
}
