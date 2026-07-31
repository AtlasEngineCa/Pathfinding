package ca.atlasengine.pathfinding.influence;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block;

import java.util.List;
import java.util.Objects;

/**
 * Restricts navigation to the union of explicitly allowed world areas.
 * Overlapping boxes can describe corridors, roads, rooms, and junctions.
 */
public final class AllowedNavigationAreas implements NavigationInfluence {
    private final List<NavigationArea> areas;

    public AllowedNavigationAreas(List<NavigationArea> areas) {
        if (areas == null || areas.isEmpty()
                || areas.stream().anyMatch(Objects::isNull)) {
            throw new IllegalArgumentException("areas cannot be empty");
        }
        this.areas = List.copyOf(areas);
    }

    public static AllowedNavigationAreas of(NavigationArea... areas) {
        if (areas == null) throw new IllegalArgumentException("areas");
        return new AllowedNavigationAreas(List.of(areas));
    }

    public List<NavigationArea> areas() {
        return areas;
    }

    @Override
    public InfluenceResult evaluate(
            Block.Getter blocks, Point point, BoundingBox box) {
        for (NavigationArea area : areas) {
            if (area.contains(point, box)) return InfluenceResult.NONE;
        }
        return InfluenceResult.forbidden("outside_allowed_navigation_areas");
    }

    @Override
    public boolean equals(Object object) {
        return object instanceof AllowedNavigationAreas other
                && areas.equals(other.areas);
    }

    @Override
    public int hashCode() {
        return areas.hashCode();
    }
}
