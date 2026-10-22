package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.internal.movement.RouteGeometry;
import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

class CornerCutPolicyTest {
    @Test
    void shorelineWaypointsAreRetained() {
        assertFalse(RouteGeometry.permitsCornerCut(TerrainType.WATER_BORDER));
    }

    @Test
    void protectedHazardAndDoorNodesCannotBeCut() {
        for (TerrainType type : new TerrainType[]{
                TerrainType.FIRE_IN_NEIGHBOR,
                TerrainType.DAMAGING_IN_NEIGHBOR,
                TerrainType.WALKABLE_DOOR
        }) {
            assertFalse(RouteGeometry.permitsCornerCut(type));
        }
    }

    @Test
    void onlyWaterBoundNavigationUsesHalfBlockVerticalTolerance() {
        assertEquals(0.5, RouteGeometry.verticalWaypointTolerance(
                NavigationMode.WATER));
        assertEquals(1.0, RouteGeometry.verticalWaypointTolerance(
                NavigationMode.FLYING));
        assertEquals(1.0, RouteGeometry.verticalWaypointTolerance(
                NavigationMode.AMPHIBIOUS));
    }
}
