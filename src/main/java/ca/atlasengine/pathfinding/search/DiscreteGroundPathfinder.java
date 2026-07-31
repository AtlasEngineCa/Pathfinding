package ca.atlasengine.pathfinding.search;

import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.internal.search.GroundNodePathfinder;
import ca.atlasengine.pathfinding.internal.search.GroundSearchSpec;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block;

import java.util.List;
import java.util.Set;
import static ca.atlasengine.pathfinding.internal.Points.finite;

/**
 * Discrete weighted-A* ground search facade.
 *
 * <p>This is deliberately separate from adaptive search: adaptive retries are
 * a distinct search strategy.</p>
 */
public final class DiscreteGroundPathfinder {
    private final GroundNodePathfinder engine = new GroundNodePathfinder();

    public PathResult findPath(Block.Getter blocks, Point start, Point target,
                               BoundingBox box, MobTraversalProfile profile,
                               GroundSearchLimits limits,
                               GroundCapabilities capabilities,
                               SearchControl control) {
        if (limits == null) throw new IllegalArgumentException("limits");
        if (!finite(start) || !finite(target)) {
            return PathResult.INVALID_REQUEST;
        }
        double maxPathLength = limits.maxPathLength();
        int baseVisited = (int) Math.floor(Math.max(16, maxPathLength) * 16);
        int visited = (int) (baseVisited * limits.maxVisitedMultiplier());
        return engine.findPathToAny(blocks, start, List.of(target), box,
                profile,
                GroundSearchSpec.walking(maxPathLength, limits.reachRange(),
                        visited, capabilities),
                control, List.of(), inferredState(blocks, start, profile));
    }

    private static EntityTraversalState inferredState(
            Block.Getter blocks, Point start, MobTraversalProfile profile) {
        boolean water = BlockTraversalData.hasWaterFluid(
                blocks.getBlock(start, Block.Getter.Condition.TYPE));
        Set<Block> standable = profile.standsOnLava()
                ? Set.of(Block.LAVA) : Set.of();
        return new EntityTraversalState(true, water, standable,
                EntityTraversalState.OVERWORLD_MIN_BUILD_HEIGHT);
    }

}
