package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.search.EntityTraversalState;import ca.atlasengine.pathfinding.search.NavigationRequest;import net.minestom.server.coordinate.ChunkRange;
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
import java.util.UUID;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;

@EnvTest
class ModularControllerE2ETest {

    @Test
    void swimmerUsesElevatedApertureWithoutClippingSolidVolume(Env env) {
        Instance instance = prepared(env);
        for (int x = -2; x <= 8; x++) {
            for (int y = 40; y <= 45; y++) {
                for (int z = -4; z <= 4; z++) {
                    instance.setBlock(x, y, z, Block.WATER);
                }
            }
        }
        List<Pos> wall = new ArrayList<>();
        for (int y = 40; y <= 45; y++) {
            for (int z = -3; z <= 3; z++) {
                if (y == 42 && z == 0) continue;
                instance.setBlock(3, y, z, Block.STONE);
                wall.add(new Pos(3, y, z));
            }
        }

        try (var service = new AsyncEntityPathfindingService(2, 16)) {
            ControlledCreature cod = spawn(
                    instance, EntityType.COD,
                    new Pos(0.5, 40.5, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(cod, service, 0.18);
            cod.controller = controller;
            controller.moveTo(new Pos(6.5, 40.5, 0.5));

            runUntilFinished(env, controller, 1_200);
            emitMovement("cod-elevated-aperture", cod, controller);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + cod.getPosition()
                            + ", nodeIndex=" + controller.nodeIndex()
                            + ", tail=" + cod.positions.subList(
                            Math.max(0, cod.positions.size() - 25),
                            cod.positions.size())
                            + ", nodes=" + controller.nodes());
            assertTrue(controller.nodes().stream().anyMatch(node ->
                            node.graphX() == 3 && node.graphY() == 42
                                    && node.graphZ() == 0),
                    () -> "swimmer did not use the aperture: "
                            + controller.nodes());
            assertTrue(cod.positions.stream().mapToDouble(Pos::y)
                            .max().orElseThrow() > 41.5,
                    "physical swimmer never ascended to the opening");
            assertTrajectoryAvoids(cod, wall);
            assertTrue(traveled(cod.positions)
                            <= pathLength(controller.nodes()) + 0.8,
                    () -> "swim follower wandered: traveled="
                            + traveled(cod.positions) + ", path="
                            + pathLength(controller.nodes()));
        }
    }

    @Test
    void waterControllerDispatchesAndFollowsThreeDimensionalSwimPath(Env env) {
        Instance instance = prepared(env);
        for (int x = -2; x <= 8; x++) {
            for (int y = 40; y <= 44; y++) {
                for (int z = -3; z <= 3; z++) {
                    instance.setBlock(x, y, z, Block.WATER);
                }
            }
        }

        try (var service = new AsyncEntityPathfindingService(2, 16)) {
            ControlledCreature cod = spawn(
                    instance, EntityType.COD, new Pos(0.5, 40.5, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(cod, service, 0.2);
            cod.controller = controller;
            Pos goal = new Pos(6.5, 43.5, 0.5);
            NavigationProfile profile = BuiltinNavigationProfiles
                    .forEntityType(EntityType.COD);
            NavigationRequest meshRequest = new NavigationRequest(
                    instance, cod.getPosition(), goal, cod.getBoundingBox(),
                    profile, 16, 1, 1, List.of(), 63,
                    new EntityTraversalState(
                            false, true, java.util.Set.of(), -64, 319,
                            List.of()),
                    16);
            controller.follow(AdaptiveMeshTestSupport.promoteAndReplay(
                    service, meshRequest));

            runUntilFinished(env, controller, 800);
            emitMovement("cod", cod, controller);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + cod.getPosition()
                            + ", nodes=" + controller.nodes());
            assertTrue(cod.getPosition().distance(
                            new Pos(6.5, 43, 0.5)) < 1.5,
                    "coordinate navigation terminates within reach range one");
            assertTrue(controller.nodes().stream().allMatch(
                    node -> node.movement() == PathNode.Movement.SWIM));
            assertTrue(controller.nodes().stream().anyMatch(
                    node -> node.blockY() > 40));
            assertEquals(7, pathLength(controller.nodes()), 1.0e-6,
                    "floor(y+0.5) start requires six horizontal and two vertical edges");
            assertTrue(traveled(cod.positions) <= 7.6,
                    "smooth follower added excessive motion between baseline swim nodes");
        }
    }

    @Test
    void flyingControllerDispatchesAndFollowsPathAroundSolidVolume(Env env) {
        Instance instance = prepared(env);
        List<Pos> wall = new ArrayList<>();
        for (int y = 40; y <= 46; y++) {
            for (int z = -1; z <= 1; z++) {
                instance.setBlock(3, y, z, Block.STONE);
                wall.add(new Pos(3, y, z));
            }
        }

        try (var service = new AsyncEntityPathfindingService(2, 16)) {
            ControlledCreature bee = spawn(
                    instance, EntityType.BEE, new Pos(0.5, 43.5, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(bee, service, 0.2);
            bee.controller = controller;
            Pos goal = new Pos(6.5, 43.5, 0.5);
            NavigationProfile profile = BuiltinNavigationProfiles
                    .forEntityType(EntityType.BEE);
            NavigationRequest meshRequest = new NavigationRequest(
                    instance, bee.getPosition(), goal, bee.getBoundingBox(),
                    profile, 48, 1, 1, List.of(), 63,
                    EntityTraversalState.GROUNDED, 48);
            controller.follow(AdaptiveMeshTestSupport.promoteAndReplay(
                    service, meshRequest));

            runUntilFinished(env, controller, 1_000);
            emitMovement("bee", bee, controller);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + bee.getPosition()
                            + ", nodes=" + controller.nodes());
            assertTrue(bee.getPosition().distance(
                            new Pos(6.5, 43, 0.5)) < 0.7,
                    "flying navigation completes against the selected block node");
            assertTrue(controller.nodes().stream().allMatch(
                    node -> node.movement() == PathNode.Movement.FLY));
            assertTrue(bee.positions.stream().anyMatch(position ->
                    position.blockY() > 46 || position.blockY() < 40
                            || Math.abs(position.blockZ()) > 1),
                    () -> "entity did not visibly clear obstacle; final path="
                            + controller.nodes());
            assertEquals(3 * Math.sqrt(2) + Math.sqrt(3) + 2,
                    pathLength(controller.nodes()), 1.0e-6,
                    "floor(y+0.5) flying start has the shortest gated 3D detour");
            assertTrue(traveled(bee.positions) <= 7.4,
                    "flying follower added excessive movement");
            assertTrajectoryAvoids(bee, wall);
        }
    }

    @Test
    void smallFlyerRecoversFromForbiddenNominalStartBeforeAsyncSearch(Env env) {
        Instance instance = prepared(env);
        instance.setBlock(0, 44, 0, Block.STONE);

        try (var service = new AsyncEntityPathfindingService(2, 16)) {
            ControlledCreature bee = spawn(instance, EntityType.BEE,
                    new Pos(0.5, 43.5, 0.5), new UUID(0, 1));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(bee, service, 0.2);
            bee.controller = controller;
            controller.moveTo(new Pos(4.5, 43.5, 0.5));

            runUntilFinished(env, controller, 800);
            emitMovement("bee-invalid-start", bee, controller);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + bee.getPosition()
                            + ", nodes=" + controller.nodes());
            PathNode first = controller.nodes().getFirst();
            assertTrue(first.graphX() != 0 || first.graphY() != 44
                            || first.graphZ() != 0,
                    () -> "worker retained forbidden nominal start: "
                            + controller.nodes());
            assertTrue(bee.getPosition().distance(
                    new Pos(4.5, 43, 0.5)) < 1.5);
        }
    }

    @Test
    void amphibiousControllerCrossesPondAndReturnsToLand(Env env) {
        Instance instance = prepared(env);
        for (int x = -3; x <= 10; x++) {
            for (int z = -4; z <= 4; z++) {
                instance.setBlock(x, 39, z, Block.STONE);
            }
        }
        for (int x = 2; x <= 6; x++) {
            for (int z = -3; z <= 3; z++) {
                instance.setBlock(x, 39, z, Block.WATER);
                instance.setBlock(x, 40, z, Block.WATER);
            }
        }

        try (var service = new AsyncEntityPathfindingService(2, 16)) {
            ControlledCreature axolotl = spawn(
                    instance, EntityType.AXOLOTL, new Pos(0.5, 40, 0.5));
            EntityNavigationController controller =
                    EntityNavigationController.builtin(axolotl, service, 0.16);
            axolotl.controller = controller;
            Pos goal = new Pos(8.5, 40, 0.5);
            NavigationProfile profile = BuiltinNavigationProfiles
                    .forEntityType(EntityType.AXOLOTL);
            NavigationRequest meshRequest = new NavigationRequest(
                    instance, axolotl.getPosition(), goal,
                    axolotl.getBoundingBox(), profile,
                    16, 1, 1, List.of(), 63,
                    EntityTraversalState.GROUNDED, 16);
            controller.follow(AdaptiveMeshTestSupport.promoteAndReplay(
                    service, meshRequest));

            runUntilFinished(env, controller, 1_200);
            emitMovement("axolotl", axolotl, controller);

            assertEquals(NavigationState.COMPLETED, controller.state(),
                    () -> "position=" + axolotl.getPosition()
                            + ", nodes=" + controller.nodes());
            assertTrue(axolotl.getPosition().distance(goal) < 1.5);
            assertTrue(controller.nodes().stream().anyMatch(node ->
                    node.blockX() >= 2 && node.blockX() <= 6));
            assertEquals(7, graphPathLength(controller.nodes()), 1.0e-6);
            assertTrue(axolotl.positions.stream().allMatch(position ->
                            position.y() >= 40 - 1.0e-3
                                    && position.y() <= 40.5 + 1.0e-3
                                    && Math.abs(position.z() - 0.5) < 1.0e-3),
                    "mixed-medium follower left the straight projected route");
            double maximumY = axolotl.positions.stream()
                    .mapToDouble(Pos::y).max().orElseThrow();
            assertTrue(maximumY > 40.4 && maximumY < 40.5,
                    "liquid-only direct advancement should smooth the projected "
                            + "water waypoints without leaving the water layer");
        }
    }

    private static Instance prepared(Env env) {
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

    private static ControlledCreature spawn(
            Instance instance, EntityType type, Pos position, UUID uuid) {
        ControlledCreature creature = new ControlledCreature(type, uuid);
        creature.setInstance(instance, position).join();
        return creature;
    }

    private static void emitMovement(String label, ControlledCreature creature,
                                     EntityNavigationController controller) {
        double traveled = 0;
        double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY;
        double minZ = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
        double maxZ = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < creature.positions.size(); i++) {
            Pos position = creature.positions.get(i);
            minX = Math.min(minX, position.x());
            minY = Math.min(minY, position.y());
            minZ = Math.min(minZ, position.z());
            maxX = Math.max(maxX, position.x());
            maxY = Math.max(maxY, position.y());
            maxZ = Math.max(maxZ, position.z());
            if (i > 0) traveled += creature.positions.get(i - 1).distance(position);
        }
        System.out.printf(java.util.Locale.ROOT,
                "E2E-AUDIT %s state=%s samples=%d traveled=%.4f "
                        + "x=[%.3f,%.3f] y=[%.3f,%.3f] z=[%.3f,%.3f] "
                        + "final=%s finalPath=%s%n",
                label, controller.state(), creature.positions.size(), traveled,
                minX, maxX, minY, maxY, minZ, maxZ,
                creature.getPosition(), controller.nodes());
    }

    private static double traveled(List<Pos> positions) {
        double result = 0;
        for (int i = 1; i < positions.size(); i++) {
            result += positions.get(i - 1).distance(positions.get(i));
        }
        return result;
    }

    private static double pathLength(List<PathNode> nodes) {
        double result = 0;
        for (int i = 1; i < nodes.size(); i++) {
            result += nodes.get(i - 1).asVec().distance(nodes.get(i).asVec());
        }
        return result;
    }

    private static double graphPathLength(List<PathNode> nodes) {
        double result = 0;
        for (int i = 1; i < nodes.size(); i++) {
            PathNode a = nodes.get(i - 1);
            PathNode b = nodes.get(i);
            int dx = b.graphX() - a.graphX();
            int dy = b.graphY() - a.graphY();
            int dz = b.graphZ() - a.graphZ();
            result += Math.sqrt(dx * dx + dy * dy + dz * dz);
        }
        return result;
    }

    private static void assertTrajectoryAvoids(
            ControlledCreature creature, List<Pos> blocks) {
        for (Pos position : creature.positions) {
            for (Pos block : blocks) {
                assertTrue(!Block.STONE.collisionShape()
                                .intersectBox(position.sub(block),
                                        creature.getBoundingBox()),
                        () -> "trajectory clipped solid block: entity="
                                + position + ", block=" + block);
            }
        }
    }

    private static void runUntilFinished(
            Env env, EntityNavigationController controller, int maximumTicks) {
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
        private final List<Pos> positions = new ArrayList<>();

        private ControlledCreature(EntityType entityType) {
            super(entityType);
        }

        private ControlledCreature(EntityType entityType, UUID uuid) {
            super(entityType, uuid);
        }

        @Override
        public void update(long time) {
            positions.add(getPosition());
            if (controller != null) controller.tick();
            super.update(time);
        }
    }
}
