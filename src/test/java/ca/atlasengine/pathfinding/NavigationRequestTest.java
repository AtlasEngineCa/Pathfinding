package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.search.EntityTraversalState;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.influence.ReturnRadiusInfluence;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class NavigationRequestTest {
    private final TestWorld world = new TestWorld();
    private final Pos start = new Pos(0.5, 1, 0.5);
    private final Pos target = new Pos(8.5, 1, 0.5);
    private final BoundingBox box = new BoundingBox(0.6, 1.8, 0.6);
    private final NavigationProfile profile =
            BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
    private final EntityTraversalState swimming =
            new EntityTraversalState(
                    false, true, Set.of(), -64, 319, List.of());

    @Test
    void builderProvidesDocumentedDefaultsAndDerivesSearchRange() {
        NavigationRequest request = NavigationRequest.builder(
                        world, start, target, box, profile)
                .maxPathLength(96)
                .build();

        assertEquals(96, request.maxPathLength());
        assertEquals(96, request.nodeSearchRange());
        assertEquals(0, request.reachRange());
        assertEquals(1, request.maxVisitedMultiplier());
        assertEquals(63, request.seaLevel());
        assertEquals(EntityTraversalState.GROUNDED, request.entityState());
        assertEquals(List.of(), request.influences());
    }

    @Test
    void builderUsesExplicitOptionsAndSnapshotsInfluences() {
        List<NavigationInfluence> influences = new ArrayList<>();
        influences.add(new ReturnRadiusInfluence(start, start, 24));

        NavigationRequest request = NavigationRequest.builder(
                        world, start, target, box, profile)
                .reachRange(2)
                .maxVisitedMultiplier(1.5)
                .influences(influences)
                .seaLevel(70)
                .entityState(swimming)
                .nodeSearchRange(40)
                .build();
        influences.clear();

        assertEquals(2, request.reachRange());
        assertEquals(1.5, request.maxVisitedMultiplier());
        assertEquals(1, request.influences().size());
        assertEquals(70, request.seaLevel());
        assertEquals(swimming, request.entityState());
        assertEquals(40, request.nodeSearchRange());
    }

    @Test
    void builderRetainsRecordValidation() {
        assertThrows(IllegalArgumentException.class,
                () -> NavigationRequest.builder(world, start, target, box, profile)
                        .maxPathLength(0)
                        .build());
        assertThrows(IllegalArgumentException.class,
                () -> NavigationRequest.builder(world, start, target, box, profile)
                        .influences(null)
                        .build());
    }

    @Test
    void rejectsBoxesThatCouldCorruptOrExplodeFootprintWork() {
        List<BoundingBox> invalid = List.of(
                new BoundingBox(0, 1, 1),
                new BoundingBox(-1, 1, 1),
                new BoundingBox(Double.NaN, 1, 1),
                new BoundingBox(Double.POSITIVE_INFINITY, 1, 1),
                new BoundingBox(65, 1, 1),
                new BoundingBox(new Vec(Double.MAX_VALUE, 0, 0),
                        new Vec(Double.MAX_VALUE, 1, 1)));

        for (BoundingBox invalidBox : invalid) {
            assertThrows(IllegalArgumentException.class,
                    () -> NavigationRequest.builder(
                            world, start, target, invalidBox, profile).build(),
                    invalidBox::toString);
        }
    }
}
