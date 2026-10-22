package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.internal.Points;
import ca.atlasengine.pathfinding.profile.GroundCapabilities;
import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.search.EntityPathfinder;
import ca.atlasengine.pathfinding.search.EntityTraversalState;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.search.SearchControl;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block;

import java.util.List;

/**
 * One ground search between two exact cells, through the public
 * {@link EntityPathfinder} entry point, for tests that pin a mob profile and
 * capabilities rather than an entity type.
 */
public final class GroundSearches {
    private static final EntityPathfinder PATHFINDER = new EntityPathfinder();

    private GroundSearches() {
    }

    /**
     * Searches from {@code start} to {@code target}. The target is taken as
     * the exact cell named, never projected onto a surface, and the entity is
     * grounded, in water only if its start cell holds water.
     *
     * @param maxVisitedMultiplier scales the expansion budget of sixteen nodes
     *                             per block of {@code maxPathLength}
     */
    public static PathResult find(Block.Getter blocks, Point start, Point target,
                                  BoundingBox box, MobTraversalProfile profile,
                                  double maxPathLength, double maxVisitedMultiplier,
                                  GroundCapabilities capabilities,
                                  SearchControl control) {
        NavigationProfile ground = NavigationProfile.builder(
                        NavigationMode.GROUND, profile, capabilities)
                .pathToTargetsBelowSurface(true)
                .build();
        // A request refuses non-finite endpoints itself; reading the world
        // first would touch it on behalf of a request that never exists.
        boolean inWater = Points.finite(start) && Points.finite(target)
                && BlockTraversalData.hasWaterFluid(
                        blocks.getBlock(start, Block.Getter.Condition.TYPE));
        NavigationRequest request = NavigationRequest.builder(
                        blocks, start, target, box, ground)
                .maxPathLength(maxPathLength)
                .maxVisitedMultiplier(maxVisitedMultiplier)
                .entityState(new EntityTraversalState(true, inWater,
                        profile.standableFluids(),
                        EntityTraversalState.OVERWORLD_MIN_BUILD_HEIGHT,
                        EntityTraversalState.OVERWORLD_MAX_BUILD_HEIGHT,
                        List.of()))
                .build();
        return PATHFINDER.findPath(request, control);
    }
}
