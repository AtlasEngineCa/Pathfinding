package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.EntityTraversalState;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.influence.NavigationZoneInfluence;
import ca.atlasengine.pathfinding.influence.NavigationArea;
import ca.atlasengine.pathfinding.influence.ReturnRadiusInfluence;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class AdversarialConfigurationTest {
    @Test
    void verticalCapabilitiesHaveFiniteWorkBounds() {
        assertThrows(IllegalArgumentException.class,
                () -> GroundCapabilities.builder().maxStepHeight(17).maxFallDistance(5).allowDiagonal(true).build());
        assertThrows(IllegalArgumentException.class,
                () -> GroundCapabilities.builder().maxStepHeight(1).maxFallDistance(1025).allowDiagonal(true).build());
        GroundCapabilities.builder().maxStepHeight(GroundCapabilities.MAX_STEP_HEIGHT).maxFallDistance(GroundCapabilities.MAX_FALL_DISTANCE).allowDiagonal(true).build();
    }

    @Test
    void entityFootprintsHaveABoundedPerNodeScan() {
        TestWorld world = new TestWorld();
        var profile = BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
        assertThrows(IllegalArgumentException.class,
                () -> NavigationRequest.builder(world,
                        new Pos(0, 1, 0), new Pos(1, 1, 0),
                        new BoundingBox(64, 64, 64), profile).build());
        NavigationRequest.builder(world,
                new Pos(0, 1, 0), new Pos(1, 1, 0),
                new BoundingBox(16, 8, 16), profile).build();
    }

    @Test
    void nonFiniteAlternateTargetsCannotCollapseOntoTheOrigin() {
        TestWorld world = new TestWorld().floor(-2, 4, -2, 2, 0,
                net.minestom.server.instance.block.Block.STONE);
        NavigationRequest request = NavigationRequest.builder(world,
                        new Pos(0.5, 1, 0.5), new Pos(3.5, 1, 0.5),
                        new BoundingBox(0.6, 1.8, 0.6),
                        BuiltinNavigationProfiles.forEntityType(
                                EntityType.ZOMBIE))
                .build();

        PathResult result = new EntityPathfinder().findPathToAny(request,
                List.of(new Pos(Double.NaN, 1, 0)), SearchControl.NONE);

        assertEquals(PathStatus.INVALID_REQUEST, result.status());
    }

    @Test
    void traversalSnapshotsCannotCreateUnboundedHeightScans() {
        assertThrows(IllegalArgumentException.class,
                () -> new EntityTraversalState(true, false, Set.of(),
                        Integer.MIN_VALUE, Integer.MAX_VALUE, List.of()));
        assertThrows(IllegalArgumentException.class,
                () -> new EntityTraversalState(true, false, Set.of(), 0,
                        EntityTraversalState.MAX_BUILD_HEIGHT_SPAN + 1,
                        List.of()));
        assertThrows(IllegalArgumentException.class,
                () -> new EntityTraversalState(true, false, Set.of(), -64,
                        319, List.of(new Pos(Double.NaN, 0, 0))));
    }

    @Test
    void nonFiniteZoneBoundsAreRejectedInsteadOfBecomingInert() {
        assertThrows(IllegalArgumentException.class,
                () -> new NavigationZoneInfluence(
                        new Pos(Double.NaN, 0, 0), new Pos(1, 1, 1),
                        true, 0, "corrupt"));
        assertThrows(IllegalArgumentException.class,
                () -> new NavigationArea(new Pos(0, 0, 0),
                        new Pos(Double.POSITIVE_INFINITY, 1, 1), "corrupt"));
        assertThrows(IllegalArgumentException.class,
                () -> new ReturnRadiusInfluence(
                        new Pos(Double.NaN, 0, 0), new Pos(0, 0, 0), 10));
    }
}
