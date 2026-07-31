package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.internal.movement.MovementContext;
import ca.atlasengine.pathfinding.internal.movement.PlatformJumpMovementExecutor;
import net.minestom.server.collision.Aerodynamics;
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
 * Live counterpart of {@link PlatformJumpTest}. Every ticked sample is
 * checked against the real registry collision shapes, so a route the planner
 * accepted but the follower cannot fly shows up as a stall, not as a pass.
 */
@EnvTest
class PlatformJumpE2ETest {
    private static final int SURFACE = 40;

    @Test
    void plannedJumpsLaunchOnceForLevelRiseAndDropGeometries(Env env) {
        Instance instance = preparedFlat(env);
        int[] lanes = {0, 12, 24};
        int[] rises = {0, 1, -1};

        for (int lane = 0; lane < lanes.length; lane++) {
            int z = lanes[lane];
            int rise = rises[lane];
            corridor(instance, z, 1, SURFACE + Math.max(0, rise));
            gap(instance, 2, 3, z, 1);
            for (int x = 4; x <= 10; x++) {
                platform(instance, x, z, 1, SURFACE + rise);
            }

            try (AsyncEntityPathfindingService service =
                         new AsyncEntityPathfindingService(1, 8)) {
                ControlledCreature mob = spawn(instance, EntityType.ZOMBIE,
                        new Pos(0.5, SURFACE, z + 0.5));
                EntityNavigationController controller =
                        new EntityNavigationController(mob, service,
                                jumpProfile(EntityType.ZOMBIE,
                                        PlatformJumpCapabilities.acrossGaps(2)
                                                .withRise(1).withDrop(1)),
                                0.18);
                mob.controller = controller;
                controller.moveTo(new Pos(8.5, SURFACE + rise, z + 0.5));

                runUntilFinished(env, controller, 1_200);

                int expected = rise;
                assertEquals(NavigationState.COMPLETED, controller.state(),
                        () -> "rise=" + expected + " position="
                                + mob.getPosition() + " nodes="
                                + controller.nodes());
                assertEquals(1, controller.nodes().stream().filter(node ->
                                node.movement() == PathNode.Movement.JUMP)
                        .count(), controller.nodes()::toString);
                assertEquals(1, mob.upwardLaunches,
                        () -> "rise=" + expected + " must launch exactly once");
                assertTrue(mob.getPosition().x() > 4.5,
                        () -> "rise=" + expected + " never reached the far "
                                + "platform: " + mob.getPosition());
                assertTrue(mob.positions.stream().mapToDouble(Pos::y).min()
                                .orElseThrow() >= SURFACE + Math.min(0, rise)
                                - 0.01,
                        () -> "rise=" + expected + " fell into the gap: "
                                + mob.getPosition());
                assertTrajectoryIsCollisionFree(mob);
                mob.remove();
            }
        }
    }

    /**
     * A rising launch climbs the whole rise plus the clearance, while the
     * chord the planner used to sweep bowed only the clearance off a rising
     * line and topped out about {@code rise/2} lower. The slabs here sit in
     * that band: the roofed lane must not be planned as a jump, and the
     * open lane must still be flown, so the refusal is the ceiling and not
     * the geometry.
     */
    @Test
    void aCeilingOnlyTheRisingLaunchReachesIsNeverJumpedInto(Env env) {
        for (boolean roofed : new boolean[]{true, false}) {
            Instance instance = preparedFlat(env);
            corridor(instance, 0, 1, SURFACE + 2);
            gap(instance, 2, 3, 0, 1);
            for (int x = 4; x <= 10; x++) {
                platform(instance, x, 0, 1, SURFACE + 2);
            }
            if (roofed) {
                instance.setBlock(3, SURFACE + 4, 0, Block.SMOOTH_STONE_SLAB
                        .withProperty("type", "top"));
            }

            try (AsyncEntityPathfindingService service =
                         new AsyncEntityPathfindingService(1, 8)) {
                ControlledCreature mob = spawn(instance, EntityType.ZOMBIE,
                        new Pos(0.5, SURFACE, 0.5));
                EntityNavigationController controller =
                        new EntityNavigationController(mob, service,
                                jumpProfile(EntityType.ZOMBIE,
                                        PlatformJumpCapabilities.builder().maxHorizontalDistance(4).maxRise(2).maxDrop(0).apexClearance(1).build()), 0.18);
                mob.controller = controller;
                controller.moveTo(new Pos(8.5, SURFACE + 2, 0.5));

                runUntilFinished(env, controller, 1_200);

                boolean roof = roofed;
                long planned = controller.nodes().stream().filter(node ->
                        node.movement() == PathNode.Movement.JUMP).count();
                assertEquals(roofed ? 0 : 1, planned,
                        () -> "roofed=" + roof + " nodes="
                                + controller.nodes());
                assertEquals(planned, mob.upwardLaunches,
                        () -> "roofed=" + roof + " planned " + planned
                                + " jumps but launched " + mob.upwardLaunches);
                assertTrue(mob.positions.stream().mapToDouble(Pos::y).min()
                                .orElseThrow() >= SURFACE - 0.01,
                        () -> "roofed=" + roof + " fell into the gap: "
                                + mob.getPosition());
                assertTrajectoryIsCollisionFree(mob);
                if (!roofed) {
                    assertEquals(NavigationState.COMPLETED, controller.state(),
                            () -> "the open lane must still be flown: "
                                    + mob.getPosition());
                    assertTrue(mob.positions.stream().mapToDouble(Pos::y).max()
                                    .orElseThrow() >= SURFACE + 2.8,
                            () -> "the launch never climbed rise plus "
                                    + "clearance: " + mob.positions);
                }
                mob.remove();
            }
        }
    }

    @Test
    void wideFootprintMobCrossesAGapWithOneLaunchAndNoCollisions(Env env) {
        Instance instance = preparedFlat(env);
        corridor(instance, 0, 2, SURFACE);
        gap(instance, 3, 5, 0, 2);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(instance, EntityType.RAVAGER,
                    new Pos(1.0, SURFACE, 1.0));
            mob.setBoundingBox(1.8, 1.8, 1.8);
            EntityNavigationController controller =
                    new EntityNavigationController(mob, service,
                            jumpProfile(EntityType.RAVAGER,
                                    // Two straddled cells plus three gap
                                    // cells of anchor-to-anchor travel.
                                    PlatformJumpCapabilities.builder().maxHorizontalDistance(5).maxRise(0).maxDrop(0).apexClearance(1).build()),
                            0.2);
            mob.controller = controller;
            controller.moveTo(new Pos(9.0, SURFACE, 1.0));

            runUntilFinished(env, controller, 1_500);

            List<PathNode> jumps = controller.nodes().stream().filter(node ->
                    node.movement() == PathNode.Movement.JUMP).toList();
            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition()
                            + ", nodes=" + controller.nodes());
            assertEquals(1, jumps.size(), controller.nodes()::toString);
            assertEquals(6, jumps.getFirst().graphX(),
                    "a two-cell footprint departs its last supported anchor");
            assertEquals(1, mob.upwardLaunches,
                    "one atomic jump edge must inject vertical velocity once");
            assertTrue(mob.getPosition().x() > 8.0,
                    () -> "wide mob never reached the far platform: "
                            + mob.getPosition());
            assertTrue(mob.positions.stream().mapToDouble(Pos::y).min()
                            .orElseThrow() >= SURFACE - 0.01,
                    () -> "wide mob fell into the gap: " + mob.positions);
            assertTrue(mob.positions.stream().mapToDouble(Pos::y).max()
                    .orElseThrow() > SURFACE + 0.5);
            assertTrajectoryIsCollisionFree(mob);
        }
    }

    /**
     * A waypoint counts as reached up to half a block from its anchor, so a
     * mob that arrives off-centre used to launch an arc the search never
     * cleared. The pillar beside the lane is missed by the planned arc and
     * struck by the off-centre one.
     */
    @Test
    void anOffCentreApproachConvergesOnTheTakeoffAndStillCrosses(Env env) {
        Instance instance = preparedFlat(env);
        corridor(instance, 0, 2, SURFACE);
        gap(instance, 2, 3, 0, 2);
        instance.setBlock(2, SURFACE, 1, Block.STONE);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(instance, EntityType.ZOMBIE,
                    new Pos(1.5, SURFACE, 0.94));
            EntityNavigationController controller =
                    new EntityNavigationController(mob, service,
                            jumpProfile(EntityType.ZOMBIE,
                                    PlatformJumpCapabilities.acrossGaps(2)),
                            0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(8.5, SURFACE, 0.5));

            runUntilFinished(env, controller, 1_200);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + mob.getPosition() + " nodes="
                            + controller.nodes());
            assertEquals(1, controller.nodes().stream().filter(node ->
                            node.movement() == PathNode.Movement.JUMP).count(),
                    controller.nodes()::toString);
            assertEquals(1, mob.upwardLaunches,
                    "an off-centre approach must still launch exactly once");
            assertTrue(mob.getPosition().x() > 4.5,
                    () -> "never reached the far platform: "
                            + mob.getPosition());
            assertTrue(mob.positions.stream().mapToDouble(Pos::y).min()
                            .orElseThrow() >= SURFACE - 0.01,
                    () -> "fell into the gap: " + mob.getPosition());
            assertTrajectoryIsCollisionFree(mob);
        }
    }

    /**
     * The scan behind the off-centre defect: every live position the
     * arrival tolerance admits has to launch the one arc the search
     * cleared, which is only true if the launch leaves the anchor itself.
     */
    @Test
    void everyOffsetInsideTheArrivalToleranceLaunchesFromTheAnchor(Env env) {
        Instance instance = preparedFlat(env);
        corridor(instance, 0, 3, SURFACE);
        gap(instance, 2, 3, 0, 3);
        Pos anchor = new Pos(1.5, SURFACE, 1.5);
        double[] offsets = {-0.44, -0.22, 0, 0.22, 0.44};
        PlatformJumpMovementExecutor executor =
                new PlatformJumpMovementExecutor();

        for (double dx : offsets) {
            for (double dz : offsets) {
                EntityCreature mob = new EntityCreature(EntityType.ZOMBIE);
                mob.setInstance(instance, anchor.add(dx, 0, dz)).join();
                RecordingContext context = new RecordingContext(mob,
                        new PathNode(anchor.x(), anchor.y(), anchor.z(),
                                PathNode.Movement.WALK, 1, SURFACE, 1),
                        new PathNode(4.5, SURFACE, 1.5,
                                PathNode.Movement.JUMP, 4, SURFACE, 1));
                Pos launchOrigin = null;
                for (int tick = 0; tick < 40 && launchOrigin == null; tick++) {
                    Pos before = mob.getPosition();
                    executor.move(context, context.current.asVec());
                    if (context.startedJumps > 0) launchOrigin = before;
                    env.tick();
                }
                Pos origin = launchOrigin;
                assertNotNull(origin,
                        () -> "offset (" + dx + ", " + dz + ") never launched");
                assertEquals(0, context.recomputes,
                        () -> "offset (" + dx + ", " + dz + ") refused a "
                                + "launch the search had cleared");
                assertEquals(0, Math.hypot(origin.x() - anchor.x(),
                                origin.z() - anchor.z()), 1.0e-6,
                        () -> "offset (" + dx + ", " + dz + ") launched from "
                                + origin + " rather than " + anchor);
                mob.remove();
            }
        }
    }

    /**
     * Convergence is not allowed to become a stall of its own: an entity
     * that cannot reach its takeoff anchor replans instead of dithering.
     */
    @Test
    void unreachableTakeoffReplansInsteadOfConvergingForever(Env env) {
        Instance instance = preparedFlat(env);
        for (int y = SURFACE; y <= SURFACE + 2; y++) {
            instance.setBlock(1, y, 0, Block.STONE);
        }
        EntityCreature mob = new EntityCreature(EntityType.ZOMBIE);
        mob.setInstance(instance, new Pos(2.5, SURFACE, 0.5)).join();
        RecordingContext context = new RecordingContext(mob,
                new PathNode(2.1, SURFACE, 0.5, PathNode.Movement.WALK,
                        2, SURFACE, 0),
                new PathNode(6.5, SURFACE, 0.5, PathNode.Movement.JUMP,
                        6, SURFACE, 0));
        PlatformJumpMovementExecutor executor =
                new PlatformJumpMovementExecutor();

        for (int tick = 0; tick < 120 && context.recomputes == 0; tick++) {
            executor.move(context, context.current.asVec());
            env.tick();
        }

        assertEquals(1, context.recomputes,
                "an unreachable takeoff must replan exactly once");
        assertTrue(context.alignmentTicks <= 41,
                () -> "convergence ran for " + context.alignmentTicks
                        + " ticks before replanning");
        assertEquals(0, context.startedJumps,
                "no launch may happen from an unconverged takeoff");
        assertTrue(mob.getPosition().x() > 2.2,
                () -> "walked through the wall: " + mob.getPosition());
        mob.remove();
    }

    /**
     * The planned arc's shoulder clips the corner of an overhang beside the
     * takeoff. A refused launch never moves the entity, so the recomputation
     * it triggers re-emits the identical edge; the entity has to be routed
     * the long way instead.
     */
    @Test
    void anArcTheFollowerCannotFlyIsNeverPlannedAndTheDetourIsTaken(Env env) {
        for (boolean overhang : new boolean[]{true, false}) {
            Instance instance = preparedFlat(env);
            trenchArena(instance, overhang);

            try (AsyncEntityPathfindingService service =
                         new AsyncEntityPathfindingService(1, 8)) {
                ControlledCreature mob = spawn(instance, EntityType.ZOMBIE,
                        new Pos(-1.5, SURFACE, 0.5));
                EntityNavigationController controller =
                        new EntityNavigationController(mob, service,
                                jumpProfile(EntityType.ZOMBIE,
                                        PlatformJumpCapabilities.builder().maxHorizontalDistance(5).maxRise(0).maxDrop(0).apexClearance(2.25).build()), 0.2);
                mob.controller = controller;
                controller.moveTo(new Pos(5.5, SURFACE, 0.5));

                runUntilFinished(env, controller, 2_000);

                long planned = controller.nodes().stream().filter(node ->
                        node.movement() == PathNode.Movement.JUMP).count();
                assertEquals(planned, mob.upwardLaunches,
                        () -> "overhang=" + overhang + " planned " + planned
                                + " jumps but launched " + mob.upwardLaunches
                                + ": " + controller.nodes());
                assertEquals(NavigationState.COMPLETED, controller.state(),
                        () -> "overhang=" + overhang + " position="
                                + mob.getPosition() + " nodes="
                                + controller.nodes());
                assertEquals(overhang ? 0 : 1, planned,
                        () -> "overhang=" + overhang + " routed over "
                                + controller.nodes());
                assertTrue(mob.positions.stream().mapToDouble(Pos::y).min()
                                .orElseThrow() >= SURFACE - 0.01,
                        () -> "overhang=" + overhang + " fell into the trench: "
                                + mob.getPosition());
                assertTrajectoryIsCollisionFree(mob);
                mob.remove();
            }
        }
    }

    /**
     * The planner never sees an entity's aerodynamics, so it keeps emitting
     * an arc a heavy entity cannot climb and a weightless one never falls
     * back from. Launching either drops the entity into the gap or drifts it
     * away; the geometric arc test cannot tell.
     */
    @ParameterizedTest
    @ValueSource(doubles = {2.0, 0.0})
    void anArcTheEntitysOwnBallisticsCannotFlyNeverLaunches(
            double gravity, Env env) {
        Instance instance = preparedFlat(env);
        corridor(instance, 0, 1, SURFACE);
        gap(instance, 2, 3, 0, 1);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(instance, EntityType.ZOMBIE,
                    new Pos(0.5, SURFACE, 0.5));
            Aerodynamics standard = mob.getAerodynamics();
            mob.setAerodynamics(new Aerodynamics(gravity,
                    standard.horizontalAirResistance(),
                    standard.verticalAirResistance()));
            EntityNavigationController controller =
                    new EntityNavigationController(mob, service,
                            jumpProfile(EntityType.ZOMBIE,
                                    PlatformJumpCapabilities.builder().maxHorizontalDistance(3).maxRise(0).maxDrop(0).apexClearance(8).build()), 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(8.5, SURFACE, 0.5));

            runUntilFinished(env, controller, 600);

            assertTrue(controller.nodes().stream().anyMatch(node ->
                            node.movement() == PathNode.Movement.JUMP),
                    () -> "the planner knows no aerodynamics and must still "
                            + "emit the edge: " + controller.nodes());
            assertEquals(0, mob.upwardLaunches,
                    "an arc the entity cannot fly must never be launched");
            assertEquals(SURFACE, mob.positions.stream()
                            .mapToDouble(Pos::y).max().orElseThrow(), 0.01,
                    () -> "gravity=" + gravity + " left the takeoff: "
                            + mob.getPosition());
            assertTrue(mob.positions.stream().mapToDouble(Pos::y).min()
                            .orElseThrow() >= SURFACE - 0.01,
                    () -> "gravity=" + gravity + " fell into the gap: "
                            + mob.getPosition());
            assertNotEquals(NavigationState.COMPLETED, controller.state());
            assertTrajectoryIsCollisionFree(mob);
        }
    }

    @Test
    void disabledCapabilityNeverLaunchesAcrossTheSameGap(Env env) {
        Instance instance = preparedFlat(env);
        corridor(instance, 0, 1, SURFACE);
        gap(instance, 2, 3, 0, 1);

        try (AsyncEntityPathfindingService service =
                     new AsyncEntityPathfindingService(1, 8)) {
            ControlledCreature mob = spawn(instance, EntityType.ZOMBIE,
                    new Pos(0.5, SURFACE, 0.5));
            EntityNavigationController controller =
                    new EntityNavigationController(mob, service,
                            jumpProfile(EntityType.ZOMBIE,
                                    PlatformJumpCapabilities.DISABLED), 0.18);
            mob.controller = controller;
            controller.moveTo(new Pos(8.5, SURFACE, 0.5));

            runUntilFinished(env, controller, 400);

            assertNotEquals(NavigationState.COMPLETED, controller.state());
            assertEquals(0, mob.upwardLaunches,
                    "a disabled capability must never launch a platform jump");
            assertTrue(controller.nodes().stream().noneMatch(node ->
                    node.movement() == PathNode.Movement.JUMP),
                    controller.nodes()::toString);
            assertTrue(mob.positions.stream().mapToDouble(Pos::y).min()
                            .orElseThrow() >= SURFACE - 0.01,
                    () -> "walker fell into the gap: " + mob.getPosition());
            assertTrajectoryIsCollisionFree(mob);
        }
    }

    /** Seals a {@code depth}-cell lane so the only route is straight ahead. */
    private static void corridor(Instance instance, int minimumZ, int depth,
                                 int topSurface) {
        for (int y = SURFACE; y <= topSurface + 3; y++) {
            for (int x = -3; x <= 13; x++) {
                instance.setBlock(x, y, minimumZ - 1, Block.STONE);
                instance.setBlock(x, y, minimumZ + depth, Block.STONE);
            }
            for (int z = minimumZ - 1; z <= minimumZ + depth; z++) {
                instance.setBlock(-3, y, z, Block.STONE);
                instance.setBlock(13, y, z, Block.STONE);
            }
        }
    }

    private static void gap(Instance instance, int minimumX, int maximumX,
                            int minimumZ, int depth) {
        for (int x = minimumX; x <= maximumX; x++) {
            for (int z = minimumZ; z < minimumZ + depth; z++) {
                for (int y = 0; y < SURFACE; y++) {
                    instance.setBlock(x, y, z, Block.AIR);
                }
            }
        }
    }

    /**
     * A sealed arena whose trench is jumpable only in the z=0 lane. The long
     * way round is roughly twice the cost, so the search prefers the jump
     * whenever it believes the arc is clear.
     */
    private static void trenchArena(Instance instance, boolean overhang) {
        for (int y = SURFACE; y <= SURFACE + 3; y++) {
            for (int x = -3; x <= 9; x++) {
                instance.setBlock(x, y, -1, Block.STONE);
                instance.setBlock(x, y, 7, Block.STONE);
            }
            for (int z = -1; z <= 7; z++) {
                instance.setBlock(-3, y, z, Block.STONE);
                instance.setBlock(9, y, z, Block.STONE);
            }
        }
        gap(instance, 1, 4, 0, 1);
        gap(instance, 1, 7, 1, 5);
        if (overhang) instance.setBlock(1, SURFACE + 4, 0, Block.STONE);
    }

    /** Rebuilds one column so its walkable surface sits at {@code surface}. */
    private static void platform(Instance instance, int x, int minimumZ,
                                 int depth, int surface) {
        for (int z = minimumZ; z < minimumZ + depth; z++) {
            for (int y = SURFACE - 2; y <= SURFACE + 2; y++) {
                instance.setBlock(x, y, z,
                        y < surface ? Block.STONE : Block.AIR);
            }
        }
    }

    private static NavigationProfile jumpProfile(
            EntityType type, PlatformJumpCapabilities jump) {
        NavigationProfile base =
                BuiltinNavigationProfiles.forEntityType(type);
        return base.withGroundCapabilities(
                base.groundCapabilities().withPlatformJump(jump));
    }

    private static Instance preparedFlat(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 3,
                (x, z) -> instance.loadChunk(x, z).join());
        return instance;
    }

    private static ControlledCreature spawn(
            Instance instance, EntityType type, Pos position) {
        ControlledCreature creature = new ControlledCreature(type);
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
        Instance instance = creature.getInstance();
        assertNotNull(instance);
        BoundingBox box = creature.getBoundingBox();
        for (Pos position : creature.positions) {
            var iterator = box.getBlocks(position);
            while (iterator.hasNext()) {
                var mutable = iterator.next();
                Pos block = new Pos(mutable.blockX(),
                        mutable.blockY(), mutable.blockZ());
                Block value = instance.getBlock(block);
                assertFalse(value.collisionShape()
                                .intersectBox(position.sub(block), box),
                        () -> "trajectory intersected " + value.key()
                                + " at " + block + " from " + position);
            }
        }
    }

    /** Drives one executor directly so the convergence budget is observable. */
    private static final class RecordingContext implements MovementContext {
        private final Entity entity;
        private final PathNode previous;
        private final PathNode current;
        private int recomputes;
        private int alignmentTicks;
        private int startedJumps;

        private RecordingContext(Entity entity, PathNode previous,
                                 PathNode current) {
            this.entity = entity;
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
            return jumpProfile(EntityType.ZOMBIE,
                    PlatformJumpCapabilities.acrossGaps(3));
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
            return ++alignmentTicks;
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
        private int upwardLaunches;

        private ControlledCreature(EntityType entityType) {
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
