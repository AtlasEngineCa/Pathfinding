package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.search.EntityTraversalState;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.search.SearchControl;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;

import java.util.List;
import java.util.Objects;

/**
 * Ground and amphibious entry points into the shared A* driver.
 */
public final class GroundNodePathfinder {
    public PathResult findAmphibiousPathToAny(
            Block.Getter blocks, Point start, List<? extends Point> targets,
            BoundingBox box, MobTraversalProfile profile,
            GroundSearchSpec spec, SearchControl control,
            List<NavigationInfluence> influences,
            EntityTraversalState entityState) {
        if (invalid(targets)) {
            return PathResult.INVALID_REQUEST;
        }
        MobTraversalProfile searchProfile = MobTraversalProfile.builder(
                        profile.name() + "_amphibious_search")
                .from(profile)
                .malus(TerrainType.WATER, 0)
                .malus(TerrainType.WALKABLE, 6)
                .malus(TerrainType.WATER_BORDER, 4)
                .build();
        List<Point> normalized = targets.stream()
                .map(target -> (Point) new Vec(
                        target.x(), Math.floor(target.y() + 0.5), target.z()))
                .toList();
        return PathSearch.find(
                new AmphibiousNodeEvaluator(blocks, start, box, searchProfile,
                        influences, entityState, spec, control),
                normalized, spec.maxPathLength(), spec.reachRange(),
                spec.maxVisited(), control);
    }

    public PathResult findPathToAny(
            Block.Getter blocks, Point start, List<? extends Point> targets,
            BoundingBox box, MobTraversalProfile profile,
            GroundSearchSpec spec, SearchControl control,
            List<NavigationInfluence> influences,
            EntityTraversalState entityState) {
        if (invalid(targets)) {
            return PathResult.INVALID_REQUEST;
        }
        return PathSearch.find(
                new GroundNodeEvaluator(blocks, start, box, profile,
                        influences, entityState, spec, control),
                targets, spec.maxPathLength(), spec.reachRange(),
                spec.maxVisited(), control);
    }

    private static boolean invalid(List<? extends Point> targets) {
        return targets == null || targets.isEmpty()
                || targets.stream().anyMatch(Objects::isNull);
    }
}
