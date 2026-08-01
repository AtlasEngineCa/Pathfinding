package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
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
import java.util.Comparator;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Registry-wide obstacle-course test for every living mob exposed by Minestom
 * version against which the library is being built.
 *
 * <p>Every course blocks the direct line. Completion alone is insufficient:
 * the accepted route must prove that it changed height or moved laterally to
 * negotiate the obstruction or mixed-medium segment.</p>
 */
@EnvTest
class EveryBuiltinMobE2ETest {
    private static final Set<EntityType> NON_MOB_LIVING_TYPES = Set.of(
            EntityType.ARMOR_STAND,
            EntityType.MANNEQUIN,
            EntityType.PLAYER);
    private static final Set<EntityType> SPATIAL_OVERRIDE_TYPES = Set.of(
            EntityType.GHAST,
            EntityType.HAPPY_GHAST);
    private static final Set<EntityType> SHAPED_GROUND_COURSE_TYPES = Set.of(
            EntityType.SHULKER);

    @Test
    void everyRegisteredMobCompletesItsBuiltInNavigatorCourse(Env env) {
        Instance instance = prepared(env);
        prepareCourses(instance);

        List<EntityType> mobs = EntityType.values().stream()
                .filter(EntityType::shouldSendAttributes)
                .filter(type -> !NON_MOB_LIVING_TYPES.contains(type))
                .sorted(Comparator.comparing(type -> type.key().asString()))
                .toList();

        assertEquals(90, mobs.size(),
                "Minestom's living-mob registry changed; every new type must "
                        + "remain in this automatically exercised matrix");

        try (var service = new AsyncEntityPathfindingService(2, 32)) {
            assertAll("every registered mob",
                    mobs.stream().map(type -> () ->
                            assertCompletes(env, instance, service, type)));
        }
    }

    private static void assertCompletes(
            Env env, Instance instance, AsyncEntityPathfindingService service,
            EntityType type) {
        NavigationProfile profile = BuiltinNavigationProfiles.forEntityType(type);
        if (SPATIAL_OVERRIDE_TYPES.contains(type)) {
            // These entities use custom flight behavior rather than installing
            // an ordinary path navigator. Exercise the library's meaningful
            // spatial implementation instead of pretending they walk.
            profile = profile.withNavigationType(NavigationMode.FLYING);
        }
        NavigationMode mode = profile.mode();
        Pos start = startFor(type, mode);
        Pos goal = goalFor(type, mode);
        TrackedCreature mob = new TrackedCreature(type);
        mob.setInstance(instance, start).join();
        EntityNavigationController controller =
                new EntityNavigationController(mob, service, profile, 0.2);
        mob.controller = controller;
        controller.moveTo(goal);

        runUntilTerminal(env, controller, 1_500);

        String label = type.key().asString() + " [" + mode + "]";
        assertEquals(NavigationState.COMPLETED, controller.state(),
                () -> label + " ended at " + mob.getPosition()
                        + " with nodes=" + controller.nodes());
        assertFalse(mob.positions.isEmpty(), label + " never received a tick");
        assertTrue(maximumDisplacement(mob.positions) > 0.5,
                () -> label + " completed without physically traversing its course");
        double completionTolerance = switch (mode) {
            case WATER, FLYING -> Math.max(1.1, type.width());
            case GROUND, AMPHIBIOUS, WALL_CLIMBER ->
                    Math.max(0.75, type.width());
        };
        assertTrue(mob.getPosition().distance(goal) <= completionTolerance,
                () -> label + " completed too far from its goal: "
                        + mob.getPosition());
        assertNonTrivialRoute(controller.nodes(), label);
        mob.remove();
    }

    private static Pos startFor(EntityType type, NavigationMode mode) {
        if (SHAPED_GROUND_COURSE_TYPES.contains(type)) {
            return new Pos(0.5, 40, 70.5);
        }
        return switch (mode) {
            case WATER -> new Pos(0.5, 41.5, 20.5);
            case FLYING -> new Pos(0.5, 44.5, 40.5);
            case AMPHIBIOUS -> new Pos(0.5, 40, 60.5);
            case GROUND, WALL_CLIMBER -> new Pos(0.5, 40, 0.5);
        };
    }

    private static Pos goalFor(EntityType type, NavigationMode mode) {
        // Keep the route inside the default 16-block search horizon, while
        // making it longer than half of even the dragon's 16-block body.
        double goalX = type.width() >= 6 ? 10.5 : 8.5;
        if (SHAPED_GROUND_COURSE_TYPES.contains(type)) {
            return new Pos(goalX, 40, 70.5);
        }
        return switch (mode) {
            case WATER -> new Pos(goalX, 43.5, 20.5);
            case FLYING -> new Pos(goalX, 46.5, 40.5);
            case AMPHIBIOUS -> new Pos(goalX, 40, 60.5);
            case GROUND, WALL_CLIMBER -> new Pos(goalX, 40, 0.5);
        };
    }

    private static void prepareCourses(Instance instance) {
        // Ground/wall-climber course: a solid wall forces a lateral detour.
        for (int y = 40; y <= 55; y++) {
            for (int z = -1; z <= 1; z++) {
                instance.setBlock(3, y, z, Block.STONE);
            }
        }

        // Water course: a floor-to-surface plug forces a 3D swimmer around
        // either side while keeping enough water for the elder guardian.
        for (int x = -4; x <= 11; x++) {
            for (int y = 39; y <= 46; y++) {
                for (int z = 12; z <= 28; z++) {
                    instance.setBlock(x, y, z, Block.WATER);
                }
            }
        }
        for (int y = 39; y <= 46; y++) {
            instance.setBlock(3, y, 20, Block.STONE);
        }

        // Flying course: a tall volume blocks the direct diagonal, but can be
        // bypassed laterally without relying on ground support.
        for (int y = 40; y <= 50; y++) {
            for (int z = 39; z <= 41; z++) {
                instance.setBlock(3, y, z, Block.STONE);
            }
        }

        // Amphibious course: replacing the floor with a pond makes every
        // direct crossing enter water; water-averse profiles must detour.
        for (int x = 2; x <= 6; x++) {
            for (int z = 58; z <= 62; z++) {
                instance.setBlock(x, 39, z, Block.WATER);
                instance.setBlock(x, 40, z, Block.WATER);
            }
        }

        // Stationary/custom-flight families do not have a meaningful ground
        // wall detour. They still receive a non-trivial shaped-ground course
        // whose half-block rise exercises search projection and live stepping.
        Block lowerSlab = Block.STONE_SLAB.withProperty("type", "bottom");
        for (int x = 3; x <= 5; x++) {
            for (int z = 66; z <= 74; z++) {
                instance.setBlock(x, 40, z, lowerSlab);
            }
        }

    }

    private static void assertNonTrivialRoute(
            List<PathNode> nodes, String label) {
        assertTrue(nodes.size() > 2,
                () -> label + " course produced a trivial route: " + nodes);
        PathNode first = nodes.getFirst();
        assertTrue(nodes.stream().anyMatch(node ->
                        node.graphY() != first.graphY()
                                || node.graphZ() != first.graphZ()
                                || node.movement() != PathNode.Movement.WALK),
                () -> label + " never deviated around or over its obstacle: "
                        + nodes);
    }

    private static double maximumDisplacement(List<Pos> positions) {
        Pos start = positions.getFirst();
        return positions.stream().mapToDouble(start::distance).max().orElse(0);
    }

    private static void runUntilTerminal(
            Env env, EntityNavigationController controller, int maximumTicks) {
        assertTimeoutPreemptively(Duration.ofSeconds(15), () -> {
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

    private static Instance prepared(Env env) {
        Instance instance = env.createFlatInstance();
        ChunkRange.chunksInRange(0, 0, 5,
                (x, z) -> instance.loadChunk(x, z).join());
        return instance;
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
