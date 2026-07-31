package ca.atlasengine.pathfinding.examples;

import net.minestom.server.instance.block.Block;
import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;
import ca.atlasengine.pathfinding.EntityNavigationController;
import ca.atlasengine.pathfinding.profile.GroundCapabilities;
import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.profile.NavigationModifiers;
import ca.atlasengine.pathfinding.NavigationOptions;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.NavigationState;
import ca.atlasengine.pathfinding.NavigationSystem;
import ca.atlasengine.pathfinding.PathDebugRenderer;
import ca.atlasengine.pathfinding.event.NavigationCompletedEvent;
import ca.atlasengine.pathfinding.event.NavigationStuckEvent;
import ca.atlasengine.pathfinding.event.RouteReplanEvent;
import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;
import ca.atlasengine.pathfinding.adaptive.SharedMeshNavigation;
import ca.atlasengine.pathfinding.adaptive.SharedMeshOptions;
import ca.atlasengine.pathfinding.adaptive.SharedMeshPolicy;
import ca.atlasengine.pathfinding.adaptive.SharedMeshPursuit;
import ca.atlasengine.pathfinding.adaptive.SharedMeshStatus;
import ca.atlasengine.pathfinding.influence.BlockAvoidanceInfluence;
import ca.atlasengine.pathfinding.influence.EntityFearInfluence;
import ca.atlasengine.pathfinding.influence.EntitySnapshot;
import ca.atlasengine.pathfinding.influence.NavigationArea;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.influence.NavigationZoneInfluence;
import ca.atlasengine.pathfinding.influence.ReturnRadiusInfluence;
import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainClassification;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;

import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Every code block printed in {@code README.md} and on the wiki, compiled.
 *
 * <p>This source set is built by {@code build}, so a snippet that stops
 * matching the API breaks the build instead of quietly misleading a reader.
 * Nothing here runs; it exists to be compiled. When a documented example
 * changes, change it here too.</p>
 *
 * @see <a href="https://github.com/AtlasEngineCa/Pathfinding/wiki">the wiki</a>
 */
@SuppressWarnings("unused")
final class DocumentedExamples {
    private DocumentedExamples() {
    }

    /** README, and wiki "Getting Started". */
    static void gettingStarted(Entity mob, Point destination) {
        NavigationSystem navigation = NavigationSystem.create();

        EntityNavigationController controller = navigation.controller(mob);
        controller.moveTo(destination);

        controller.tick();

        controller.close();
        navigation.close();
    }

    /** Wiki, "Start here": reading controller state. */
    static void readingState(EntityNavigationController controller, Entity mob) {
        switch (controller.state()) {
            case COMPUTING, FOLLOWING -> {}
            case COMPLETED -> onArrived(mob);
            case PARTIAL -> {
                if (controller.targetUnreachable()) chooseAnotherGoal(mob);
            }
            case STUCK -> unwedge(mob);
            case IDLE, FAILED, CANCELLED -> {}
        }
    }

    /** Wiki, "Tuning the system". */
    static NavigationOptions tuning() {
        try (NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(8)
                .queueCapacity(256)
                .movementPerTick(0.2)
                .build()) {
            return navigation.options();
        }
    }

    /** Wiki, "Modifiers". */
    static void modifiers(EntityNavigationController controller, Entity owner) {
        NavigationModifiers following = NavigationModifiers.builder()
                .terrainCost(TerrainType.WATER, 0)
                .build();

        controller.moveTo(owner.getPosition(), following);
    }

    /** Wiki, "Influences". */
    static void influences(EntityNavigationController controller, Point target,
                           List<EntitySnapshot> threatSnapshots,
                           Point firstBlock, Point lastBlock,
                           Point home, Point start) {
        List<NavigationInfluence> influences = List.of(
                new EntityFearInfluence(threatSnapshots, 8, 2, 20),
                new BlockAvoidanceInfluence(Set.of(Block.CACTUS), 1, true, 0),
                NavigationZoneInfluence.blocks(
                        firstBlock, lastBlock, true, 0, "temporary-danger"),
                new ReturnRadiusInfluence(home, start, 32));

        controller.moveTo(target, influences, NavigationModifiers.NONE);

        NavigationZoneInfluence closed = NavigationZoneInfluence.blocks(
                new Vec(4, 64, 4), new Vec(6, 66, 6), true, 0, "closed-road");
        NavigationArea plaza = NavigationArea.blocks(
                new Vec(4, 64, 4), new Vec(6, 66, 6), "plaza");
    }

    /** Wiki, "Custom mobs". */
    static EntityNavigationController customMob(
            NavigationSystem navigation, Entity entity) {
        TerrainClassification moddedTerrain = TerrainClassification.ofBlocks(
                Map.of(Block.SCULK, TerrainType.DAMAGING,
                        Block.SLIME_BLOCK, TerrainType.STICKY_HONEY));

        MobTraversalProfile ashcrawler = MobTraversalProfile.builder("ashcrawler")
                .from(MobTraversalProfile.STRIDER)
                .malus(TerrainType.DAMAGING, 4)
                .canOpenDoors(true)
                .classification(moddedTerrain)
                .build();

        GroundCapabilities body = GroundCapabilities.builder()
                .platformJump(PlatformJumpCapabilities.acrossGaps(3))
                .climbables(ClimbableCapabilities.STANDARD)
                .build();

        NavigationProfile profile =
                NavigationProfile.builder(NavigationMode.GROUND, ashcrawler, body)
                        .avoidSun(true)
                        .build();

        return navigation.controller(entity, profile, 0.2);
    }

    /** Wiki, "Platform jumping", "Planned climbing", "Doors". */
    static void capabilities(NavigationProfile base,
                             EntityNavigationController controller,
                             Point target) {
        PlatformJumpCapabilities jump = PlatformJumpCapabilities
                .acrossGaps(2)
                .withRise(1)
                .withDrop(2);

        NavigationProfile jumping = base.withGroundCapabilities(
                base.groundCapabilities().withPlatformJump(jump));

        controller.moveTo(target, NavigationModifiers.builder()
                .platformJump(jump)
                .build());

        controller.moveTo(target, NavigationModifiers.builder()
                .climbables(ClimbableCapabilities.bothDirections(1.0))
                .build());

        MobTraversalProfile polite = MobTraversalProfile.builder("polite")
                .from(base.mobProfile())
                .blockManipulation(
                        BlockManipulationCapabilities.STANDARD.closingBehind())
                .build();
    }

    /** Wiki, "The shared mesh". */
    static void sharedMesh(Entity mob, Entity target,
                           long worldRevision, long tick) {
        try (NavigationSystem navigation = NavigationSystem.builder()
                .sharedMesh(SharedMeshOptions.enabledWith(
                        SharedMeshPolicy.builder()
                                .promotionRequests(16)
                                .regionSize(64)
                                .maximumRegions(128)
                                .build()))
                .build()) {
            SharedMeshNavigation mesh = navigation.sharedMesh();

            SharedMeshPursuit pursuit =
                    mesh.pursue(mob, target, worldRevision, tick);
            pursuit.tick(worldRevision, tick);
            NavigationState state = pursuit.state();
            pursuit.close();

            SharedMeshStatus status = mesh.status();
            boolean full = status.regionsExhausted();
            mesh.tick(tick);
        }
    }

    /** Wiki, "Start here": listeners, block watching, debug render. */
    static void callbacksAndExtras(NavigationSystem navigation, Entity mob,
                                   net.minestom.server.instance.Instance instance) {
        navigation.eventNode()
                .addListener(NavigationCompletedEvent.class,
                        e -> onArrived(e.getEntity()))
                .addListener(NavigationStuckEvent.class,
                        e -> unwedge(e.getEntity()))
                .addListener(RouteReplanEvent.class,
                        e -> log(e.reason()));

        navigation.watchBlockChanges(instance);

        PathDebugRenderer renderer = new PathDebugRenderer();
        renderer.render(navigation.controller(mob));
    }

    private static void log(RouteReplanEvent.Reason reason) {
    }

    private static void onArrived(Entity mob) {
    }

    private static void chooseAnotherGoal(Entity mob) {
    }

    private static void unwedge(Entity mob) {
    }
}
