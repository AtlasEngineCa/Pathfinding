package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.influence.AllowedNavigationAreas;
import ca.atlasengine.pathfinding.influence.InfluenceResult;
import ca.atlasengine.pathfinding.influence.NavigationArea;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class AllowedNavigationAreasTest {
    @Test
    void overlappingAreasFormARestrictedWalkablePath() {
        TestWorld world = new TestWorld().floor(-5, 20, -8, 8, 0, Block.STONE);
        AllowedNavigationAreas road = AllowedNavigationAreas.of(
                new NavigationArea(new Vec(-2, 0, -1),
                        new Vec(8, 4, 2), "east-road"),
                new NavigationArea(new Vec(7, 0, 1),
                        new Vec(10, 4, 7), "turn"),
                new NavigationArea(new Vec(9, 0, 5),
                        new Vec(17, 4, 8), "north-road"));
        BoundingBox box = new BoundingBox(0.8, 1.8, 0.8);
        NavigationProfile profile = BuiltinNavigationProfiles.forEntityType(
                EntityType.ZOMBIE);
        NavigationRequest request = NavigationRequest.builder(world,
                        new Pos(0.5, 1, 0.5), new Pos(15.5, 1, 6.5),
                        box, profile)
                .maxPathLength(40)
                .maxVisitedMultiplier(8)
                .influences(List.of(road))
                .build();

        PathResult result = new EntityPathfinder().findPath(
                request, SearchControl.NONE);

        assertTrue(result.found(), result::toString);
        assertTrue(result.nodes().stream().allMatch(node -> road.evaluate(
                world, node.asVec(), box).equals(InfluenceResult.NONE)));
        assertTrue(result.nodes().stream().anyMatch(node -> node.graphZ() >= 5),
                "route did not follow the L-shaped allowed road");
    }

    @Test
    void completeLargeBoundingBoxMustFitInsideArea() {
        AllowedNavigationAreas narrow = AllowedNavigationAreas.of(
                new NavigationArea(new Vec(0, 0, 0),
                        new Vec(10, 4, 1), "one-wide"));
        assertFalse(narrow.evaluate(new TestWorld(), new Vec(5, 1, 0.5),
                new BoundingBox(1.8, 1.8, 1.8)).equals(InfluenceResult.NONE));
    }
}
