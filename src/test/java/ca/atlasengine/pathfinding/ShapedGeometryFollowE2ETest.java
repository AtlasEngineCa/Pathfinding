package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
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

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Live counterpart to the planner sweep. Every fixture sits in a sealed
 * one-wide corridor at x=3, so a run that reports progress past it must have
 * physically stood on or in that cell.
 *
 * <p>Widen the live grid with {@code -Dpathfinding.shapedFollowScenarios=all}.
 * The default grid keeps one width per axis.</p>
 */
@EnvTest
class ShapedGeometryFollowE2ETest {
    private static final String GRID_PROPERTY =
            "pathfinding.shapedFollowScenarios";
    private static final int SURFACE = 40;
    private static final int LANE = 3;
    private static final String[] FACINGS =
            {"north", "south", "east", "west"};

    private record Run(NavigationState state, Pos end, List<Pos> track) {}

    private static boolean wideGrid() {
        return "all".equals(System.getProperty(GRID_PROPERTY));
    }

    @Test
    void everyRiseUpToHalfABlockIsWalkedOntoItsOwnSurface(Env env) {
        record Case(Block fixture, double surface) {}
        List<Case> cases = new ArrayList<>();
        for (int layers = 1; layers <= 5; layers++) {
            Block snow = Block.SNOW.withProperty(
                    "layers", String.valueOf(layers));
            cases.add(new Case(snow, top(snow)));
        }
        cases.add(new Case(Block.STONE_SLAB.withProperty("type", "bottom"),
                0.5));
        cases.add(new Case(Block.WHITE_CARPET, top(Block.WHITE_CARPET)));

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            for (Case scenario : cases) {
                Instance instance = corridor(env);
                instance.setBlock(LANE, SURFACE, 0, scenario.fixture());
                Run run = drive(env, service, instance, 0.6);
                assertEquals(NavigationState.COMPLETED, run.state(),
                        () -> "did not cross " + scenario.fixture()
                                + ", ended at " + run.end());
                assertTrue(run.end().x() > 5.0,
                        () -> "stopped short of the goal: " + run.end());
                double stood = highestInLane(run);
                assertTrue(stood >= SURFACE + scenario.surface() - 0.02,
                        () -> "never stood on " + scenario.fixture()
                                + ": highest lane y=" + stood + ", surface="
                                + (SURFACE + scenario.surface()));
            }
        }
    }

    @Test
    void anUnshapedFullBlockRiseIsClearedByTheAscentImpulse(Env env) {
        Block[] fixtures = {Block.STONE, Block.COBBLESTONE, Block.OAK_PLANKS};
        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            for (Block fixture : fixtures) {
                Instance instance = corridor(env);
                instance.setBlock(LANE, SURFACE, 0, fixture);
                Run run = drive(env, service, instance, 0.6);
                assertEquals(NavigationState.COMPLETED, run.state(),
                        () -> "did not climb " + fixture + " at " + run.end());
                assertTrue(highestInLane(run) >= SURFACE + 1.0,
                        () -> "never reached the top of " + fixture
                                + ": " + highestInLane(run));
            }
        }
    }

    @Test
    void aBlockedHeadCellIsNeverEnteredForAnyShape(Env env) {
        List<Block> heads = new ArrayList<>();
        heads.add(Block.STONE);
        for (String type : new String[]{"top", "bottom", "double"}) {
            heads.add(Block.STONE_SLAB.withProperty("type", type));
        }
        for (int layers = 5; layers <= 8; layers++) {
            heads.add(Block.SNOW.withProperty(
                    "layers", String.valueOf(layers)));
        }
        for (String facing : FACINGS) {
            heads.add(Block.OAK_STAIRS.withProperty("facing", facing)
                    .withProperty("half", "bottom")
                    .withProperty("shape", "straight"));
        }
        assertEquals(12, heads.size());

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            for (Block head : heads) {
                Instance instance = corridor(env);
                instance.setBlock(LANE, SURFACE + 1, 0, head);
                Run run = drive(env, service, instance, 0.6);
                assertNotEquals(NavigationState.COMPLETED, run.state(),
                        () -> "claimed to pass under " + head);
                assertTrue(run.end().x() < LANE,
                        () -> "entered a blocked head cell under " + head
                                + " at " + run.end());
                assertFalse(occupied(instance, run.end(),
                                new BoundingBox(0.6, 1.95, 0.6)),
                        () -> "wedged under " + head + " at " + run.end());
            }
        }
    }

    @Test
    void everyShapedLaneSettlesAndACompletedRunIsCleanAtTheGoal(Env env) {
        List<Block> fixtures = new ArrayList<>();
        for (String facing : FACINGS) {
            for (String half : new String[]{"bottom", "top"}) {
                for (String shape : wideGrid()
                        ? new String[]{"straight", "inner_left",
                        "inner_right", "outer_left", "outer_right"}
                        : new String[]{"straight", "inner_left"}) {
                    fixtures.add(Block.OAK_STAIRS
                            .withProperty("facing", facing)
                            .withProperty("half", half)
                            .withProperty("shape", shape));
                }
            }
            for (String half : new String[]{"bottom", "top"}) {
                for (String open : new String[]{"false", "true"}) {
                    fixtures.add(Block.OAK_TRAPDOOR
                            .withProperty("facing", facing)
                            .withProperty("half", half)
                            .withProperty("open", open));
                }
            }
        }
        for (String type : new String[]{"top", "bottom", "double"}) {
            fixtures.add(Block.STONE_SLAB.withProperty("type", type));
        }
        for (int layers = 1; layers <= 8; layers++) {
            fixtures.add(Block.SNOW.withProperty(
                    "layers", String.valueOf(layers)));
        }
        assertEquals(wideGrid() ? 67 : 43, fixtures.size());

        BoundingBox box = new BoundingBox(0.6, 1.95, 0.6);
        int completed = 0;
        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            for (Block fixture : fixtures) {
                for (int dy : new int[]{0, 1}) {
                    Instance instance = corridor(env);
                    instance.setBlock(LANE, SURFACE + dy, 0, fixture);
                    Run run = drive(env, service, instance, 0.6);
                    assertTrue(terminal(run.state()),
                            () -> "never settled on " + fixture + " at dy="
                                    + " " + run.state());
                    if (run.state() != NavigationState.COMPLETED) continue;
                    completed++;
                    assertTrue(run.end().x() > 5.0,
                            () -> "COMPLETED away from the goal past "
                                    + fixture + ": " + run.end());
                    assertFalse(occupied(instance, run.end(), box),
                            () -> "COMPLETED inside a collision shape past "
                                    + fixture + ": " + run.end());
                    assertTrue(run.track().stream().anyMatch(
                                    p -> p.blockX() == LANE),
                            () -> "COMPLETED without entering the lane of "
                                    + fixture);
                }
            }
        }
        // Most of this grid is currently refused rather than crossed. The
        // floor only guards the sweep against degenerating into a fixture
        // that never reaches the follower at all.
        int crossed = completed;
        assertTrue(crossed >= 12,
                () -> "only " + crossed + " of " + fixtures.size() * 2
                        + " shaped lanes were crossed; the grid stopped "
                        + "exercising the follower");
    }

    @Test
    void aTallPostRaisesTheWalkedSurfaceByHalfABlock(Env env) {
        List<Block> posts = new ArrayList<>();
        posts.add(Block.OAK_FENCE);
        for (String north : new String[]{"none", "low", "tall"}) {
            for (String east : new String[]{"none", "low", "tall"}) {
                posts.add(Block.COBBLESTONE_WALL
                        .withProperty("north", north)
                        .withProperty("south", north)
                        .withProperty("east", east)
                        .withProperty("west", east)
                        .withProperty("up", "true"));
            }
        }
        assertEquals(10, posts.size());

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            for (Block post : posts) {
                Instance instance = corridor(env);
                for (int x = 2; x <= 5; x++) {
                    instance.setBlock(x, SURFACE - 1, 0, post);
                }
                Run run = drive(env, service, instance, 0.6);
                assertEquals(NavigationState.COMPLETED, run.state(),
                        () -> "did not cross the " + post.key().value()
                                + " walkway: " + run.end());
                double onPost = run.track().stream()
                        .filter(p -> p.blockX() >= 2 && p.blockX() <= 5)
                        .mapToDouble(Pos::y).max().orElse(-1);
                assertTrue(onPost >= SURFACE + 0.5 - 0.02,
                        () -> "never stood on the post tops of "
                                + post.key().value() + ": " + onPost);
            }
        }
    }

    private static double top(Block block) {
        return block.collisionShape().relativeEnd().y();
    }

    private double highestInLane(Run run) {
        return run.track().stream().filter(p -> p.blockX() == LANE)
                .mapToDouble(Pos::y).max().orElse(-1);
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

    private Run drive(Env env, AsyncEntityPathfindingService service,
                      Instance instance, double width) {
        Tracked mob = new Tracked();
        mob.setBoundingBox(width, 1.95, width);
        mob.setInstance(instance, new Pos(0.5, SURFACE, 0.5)).join();
        EntityNavigationController controller =
                new EntityNavigationController(mob, service,
                        NavigationProfile.builder(NavigationMode.GROUND, MobTraversalProfile.DEFAULT, GroundCapabilities.STANDARD).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build(), 0.2);
        mob.controller = controller;
        controller.moveTo(new Pos(6.5, SURFACE, 0.5));
        int settled = 0;
        for (int tick = 0; tick < 260 && settled < 25; tick++) {
            env.tick();
            settled = terminal(controller.state()) ? settled + 1 : 0;
            if (controller.state() == NavigationState.COMPUTING) {
                try {
                    Thread.sleep(1);
                } catch (InterruptedException interrupted) {
                    Thread.currentThread().interrupt();
                }
            }
        }
        Run run = new Run(controller.state(), mob.getPosition(),
                List.copyOf(mob.track));
        mob.remove();
        return run;
    }

    /** Sealed one-wide corridor: a refused cell has no detour around it. */
    private static Instance corridor(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 2,
                (x, z) -> instance.loadChunk(x, z).join());
        for (int x = -2; x <= 10; x++) {
            for (int y = SURFACE; y <= SURFACE + 4; y++) {
                instance.setBlock(x, y, 0, Block.AIR);
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 1, Block.STONE);
            }
            instance.setBlock(x, SURFACE - 1, 0, Block.STONE);
        }
        for (int y = SURFACE; y <= SURFACE + 4; y++) {
            instance.setBlock(-1, y, 0, Block.STONE);
            instance.setBlock(8, y, 0, Block.STONE);
        }
        return instance;
    }

    private static final class Tracked extends EntityCreature {
        private final List<Pos> track = new ArrayList<>();
        private EntityNavigationController controller;

        private Tracked() {
            super(EntityType.ZOMBIE);
        }

        @Override
        public void update(long time) {
            if (controller != null) controller.tick();
            super.update(time);
            track.add(getPosition());
        }
    }
}
