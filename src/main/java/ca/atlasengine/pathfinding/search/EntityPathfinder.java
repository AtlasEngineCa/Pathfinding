package ca.atlasengine.pathfinding.search;

import ca.atlasengine.pathfinding.internal.LoadedBlocks;
import ca.atlasengine.pathfinding.internal.search.GroundNodePathfinder;
import ca.atlasengine.pathfinding.internal.search.GroundSearchSpec;
import ca.atlasengine.pathfinding.profile.GroundCapabilities;
import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;

import java.util.List;

import static ca.atlasengine.pathfinding.internal.Points.allFinite;

/**
 * Dispatches an entity navigation request to the selected movement model.
 */
public final class EntityPathfinder {
    private final GroundNodePathfinder groundNodes = new GroundNodePathfinder();
    private final SpatialPathfinder spatial = new SpatialPathfinder();

    public PathResult findPath(NavigationRequest request, SearchControl control) {
        return findPathToAny(request, List.of(request.target()), control);
    }

    /**
     * Searches once for any destination using one shared work budget.
     */
    public PathResult findPathToAny(
            NavigationRequest request, List<? extends Point> destinations,
            SearchControl control) {
        if (!allFinite(destinations)) return PathResult.INVALID_REQUEST;
        NavigationRequest loaded =
                request.withBlocks(LoadedBlocks.view(request.blocks()));
        PathResult result = switch (request.profile().mode()) {
            case WATER, FLYING ->
                    spatial.findPathToAny(loaded, destinations, control);
            case GROUND, WALL_CLIMBER, AMPHIBIOUS ->
                    findGround(loaded, destinations, control);
        };
        return LoadedBlocks.qualify(loaded.blocks(), result);
    }

    private PathResult findGround(
            NavigationRequest request, List<? extends Point> destinations,
            SearchControl control) {
        GroundCapabilities capabilities = request.profile().groundCapabilities();
        int visited = request.maxVisitedNodes();
        MobTraversalProfile mobProfile = snapshotRailState(request);
        if (request.profile().mode() == NavigationMode.AMPHIBIOUS) {
            GroundSearchSpec spec = GroundSearchSpec.amphibious(
                    request.maxPathLength(), request.reachRange(), visited,
                    capabilities,
                    request.profile().mobProfile()
                            .prefersJumpToPreferredBlocks(),
                    request.profile().prefersShallowWater(),
                    request.seaLevel());
            return groundNodes.findAmphibiousPathToAny(
                    request.blocks(), request.start(), destinations,
                    request.boundingBox(), mobProfile, spec, control,
                    request.influences(), request.entityState());
        }
        List<? extends Point> surfaceTargets = request.profile()
                .pathToTargetsBelowSurface()
                ? destinations
                : destinations.stream().map(destination ->
                normalizeGroundTarget(request, destination)).toList();
        return groundNodes.findPathToAny(
                request.blocks(), request.start(), surfaceTargets,
                request.boundingBox(), mobProfile,
                GroundSearchSpec.walking(request.maxPathLength(),
                        request.reachRange(), visited, capabilities),
                control, request.influences(), request.entityState());
    }

    private static Point normalizeGroundTarget(
            NavigationRequest request, Point destination) {
        int x = destination.blockX();
        int y = destination.blockY();
        int z = destination.blockZ();
        Block.Getter blocks = request.blocks();
        EntityTraversalState state = request.entityState();
        Block block = blocks.getBlock(x, y, z, Block.Getter.Condition.TYPE);
        if (block.air()) {
            int cursor = y - 1;
            while (cursor >= state.minBuildHeight()
                    && blocks.getBlock(x, cursor, z,
                    Block.Getter.Condition.TYPE).air()) cursor--;
            if (cursor >= state.minBuildHeight()) {
                y = cursor + 1;
            } else {
                cursor = destination.blockY() + 1;
                while (cursor <= state.maxBuildHeight()
                        && blocks.getBlock(x, cursor, z,
                        Block.Getter.Condition.TYPE).air()) cursor++;
                y = cursor;
            }
            block = blocks.getBlock(x, y, z, Block.Getter.Condition.TYPE);
        }
        if (block.solid()) {
            do {
                y++;
                if (y > state.maxBuildHeight()) break;
                block = blocks.getBlock(x, y, z, Block.Getter.Condition.TYPE);
            } while (block.solid());
        }
        return new Vec(x, y, z);
    }

    private static MobTraversalProfile snapshotRailState(NavigationRequest request) {
        MobTraversalProfile profile = request.profile().mobProfile();
        if (profile.currentlyOnRail()) return profile;
        if (!BlockTraversalData.isRailAt(request.blocks(), request.start())) {
            return profile;
        }
        return MobTraversalProfile.builder(profile.name() + "_on_rail")
                .from(profile)
                .currentlyOnRail(true)
                .build();
    }
}
