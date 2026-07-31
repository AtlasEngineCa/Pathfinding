package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.search.EntityTraversalState;
import ca.atlasengine.pathfinding.search.SearchControl;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.terrain.BlockTagIndex;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block;

import java.util.List;

/**
 * Ground evaluator variant that also opens the vertical water neighbours a
 * swimming land mob needs.
 */
final class AmphibiousNodeEvaluator extends GroundNodeEvaluator {
    private final boolean prefersJumpToPreferredBlocks;
    private final boolean prefersShallowWater;
    private final int seaLevel;

    AmphibiousNodeEvaluator(Block.Getter blocks, Point startPosition,
                            BoundingBox box, MobTraversalProfile profile,
                            List<NavigationInfluence> influences,
                            EntityTraversalState entityState,
                            GroundSearchSpec spec, SearchControl control) {
        super(blocks, startPosition, box, profile, influences, entityState,
                spec, control);
        this.prefersJumpToPreferredBlocks = spec.prefersJumpToPreferredBlocks();
        this.prefersShallowWater = spec.prefersShallowWater();
        this.seaLevel = spec.seaLevel();
    }

    @Override
    boolean isAmphibious() {
        return true;
    }

    @Override
    SearchNode getStart() {
        if (!entityState.inWater()) return super.getStart();
        int x = (int) Math.floor(startPosition.x() + box.minX());
        int y = (int) Math.floor(startPosition.y() + box.minY()
                + (prefersJumpToPreferredBlocks ? 0 : 0.5));
        int z = (int) Math.floor(startPosition.z() + box.minZ());
        return accepted(x, y, z);
    }

    @Override
    void addVerticalNeighbors(SearchNode current, List<SearchNode> result,
                              int jumpSize, double floor) {
        SearchNode up = findAccepted(current.x, current.y + 1, current.z,
                Math.max(0, jumpSize - 1), floor,
                new Direction(0, 0), current.type);
        SearchNode down = findAccepted(current.x, current.y - 1, current.z,
                jumpSize, floor, new Direction(0, 0), current.type);
        if (validVertical(up, current)) result.add(up);
        if (current.type != TerrainType.TRAPDOOR
                && validVertical(down, current)) result.add(down);
        if (prefersShallowWater) {
            for (SearchNode neighbor : result) {
                if (neighbor.type == TerrainType.WATER
                        && neighbor.y < seaLevel - 10) {
                    neighbor.malus++;
                }
            }
        }
    }

    private boolean validVertical(SearchNode vertical, SearchNode current) {
        return validCardinal(vertical, current)
                && vertical.type == TerrainType.WATER;
    }

    @Override
    TerrainType classify(int x, int y, int z) {
        if (prefersJumpToPreferredBlocks && BlockTagIndex.contains(
                "frog_prefer_jump_to",
                blocks.getBlock(x, y - 1, z,
                        Block.Getter.Condition.TYPE))) {
            return TerrainType.OPEN;
        }
        return classifyAmphibious(x, y, z);
    }
}
