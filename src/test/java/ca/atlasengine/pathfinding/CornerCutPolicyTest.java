package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.internal.movement.RouteGeometry;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.entity.EntityType;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class CornerCutPolicyTest {
    @Test
    void frogRetainsWaterBorderWaypointWhileOtherMobsMayCutIt() {
        assertFalse(RouteGeometry.permitsCornerCut(
                EntityType.FROG, TerrainType.WATER_BORDER));
        assertTrue(RouteGeometry.permitsCornerCut(
                EntityType.AXOLOTL, TerrainType.WATER_BORDER));
    }

    @Test
    void protectedHazardAndDoorNodesCannotBeCut() {
        for (TerrainType type : new TerrainType[]{
                TerrainType.FIRE_IN_NEIGHBOR,
                TerrainType.DAMAGING_IN_NEIGHBOR,
                TerrainType.WALKABLE_DOOR
        }) {
            assertFalse(RouteGeometry.permitsCornerCut(
                    EntityType.ZOMBIE, type));
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
