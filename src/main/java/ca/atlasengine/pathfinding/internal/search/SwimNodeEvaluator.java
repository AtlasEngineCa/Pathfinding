package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.search.SearchControl;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.TerrainClassification;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;

import java.util.List;

/**
 * Volume evaluator for swimming movement.
 */
public final class SwimNodeEvaluator extends SpatialNodeEvaluator {
    private static final int[][] SWIM_DIRECTIONS = buildSwimDirections();
    private static final int[][] SWIM_DIAGONAL_GATES =
            {{2, 5}, {3, 5}, {3, 4}, {2, 4}};
    private final List<SearchNode> neighbors = new java.util.ArrayList<>(10);
    private final SearchNode[] axialNeighbors = new SearchNode[6];

    public SwimNodeEvaluator(NavigationRequest request, SearchControl control) {
        super(request, control);
    }

    SwimNodeEvaluator(NavigationRequest request) {
        this(request, SearchControl.NONE);
    }

    @Override
    SearchNode getStart() {
        return node(startPoint(request));
    }

    @Override
    int lastExaminedNeighbors() {
        return SWIM_DIRECTIONS.length;
    }

    @Override
    PathNode.Movement movement(SearchNode node) {
        return node.type == TerrainType.BREACH
                ? PathNode.Movement.BREACH : PathNode.Movement.SWIM;
    }

    @Override
    SearchNode accepted(Point point) {
        TerrainType type = cachedType(point);
        boolean permitted = type == TerrainType.WATER
                || type == TerrainType.BREACH
                && request.profile().allowBreaching();
        float base = (float) mobProfile.malus(type);
        SearchNode node = node(point);
        if (!permitted || base < 0) {
            node.malus = -1;
            return null;
        }
        node.type = type;
        node.malus = Math.max(node.malus, base);
        if (!BlockTraversalData.hasWaterFluid(request.blocks().getBlock(
                point, Block.Getter.Condition.TYPE))) {
            node.malus = (float) (node.malus + 8);
        }
        return node;
    }

    @Override
    List<SearchNode> getNeighbors(SearchNode current) {
        neighbors.clear();
        for (int i = 0; i < 6; i++) {
            int[] direction = SWIM_DIRECTIONS[i];
            Point point = point(current).add(
                    direction[0], direction[1], direction[2]);
            axialNeighbors[i] = accepted(point);
            if (axialNeighbors[i] != null && !axialNeighbors[i].settled) {
                neighbors.add(axialNeighbors[i]);
            }
        }
        // SWIM_DIRECTIONS diagonals are NE, SE, SW, NW. Their axial
        // prerequisites in the first six entries are N/E, S/E, S/W, N/W.
        for (int i = 0; i < SWIM_DIAGONAL_GATES.length; i++) {
            SearchNode first = axialNeighbors[SWIM_DIAGONAL_GATES[i][0]];
            SearchNode second = axialNeighbors[SWIM_DIAGONAL_GATES[i][1]];
            if (first == null || second == null
                    || first.malus < 0 || second.malus < 0
                    || combinedInfluence(point(first)).blocked()
                    || combinedInfluence(point(second)).blocked()) {
                continue;
            }
            int[] direction = SWIM_DIRECTIONS[6 + i];
            SearchNode diagonal = accepted(point(current).add(
                    direction[0], direction[1], direction[2]));
            if (diagonal != null && !diagonal.settled) neighbors.add(diagonal);
        }
        return neighbors;
    }

    TerrainType swimType(Point point) {
        Block.Getter blocks = request.blocks();
        BoundingBox box = request.boundingBox();
        TerrainClassification classification = mobProfile.classification();
        int sizeX = Math.max(1, (int) Math.floor(box.width() + 1));
        int sizeY = Math.max(1, (int) Math.floor(box.height() + 1));
        int sizeZ = Math.max(1, (int) Math.floor(box.depth() + 1));
        int minX = point.blockX();
        int minY = point.blockY();
        int minZ = point.blockZ();
        Block last = Block.AIR;
        boolean lastNamedWater = false;
        int scanned = 0;
        for (int x = minX; x < minX + sizeX; x++) {
            for (int y = minY; y < minY + sizeY; y++) {
                for (int z = minZ; z < minZ + sizeZ; z++) {
                    if (control.interruptible() && (scanned++ & 63) == 0
                            && (control.cancelled() || control.timedOut())) {
                        return TerrainType.BLOCKED;
                    }
                    Block block = blocks.getBlock(
                            x, y, z, Block.Getter.Condition.TYPE);
                    // A named block answers for its own cell: WATER makes it
                    // swimmable and every other type keeps the volume out,
                    // which is what an unswimmable type already means here.
                    TerrainType named = classification
                            == TerrainClassification.NONE
                            ? null : classification.classify(block);
                    if (named != null) {
                        if (named != TerrainType.WATER) return named;
                        last = block;
                        lastNamedWater = true;
                        continue;
                    }
                    Block below = blocks.getBlock(
                            x, y - 1, z, Block.Getter.Condition.TYPE);
                    if (!BlockTraversalData.hasWaterFluid(block)) {
                        if (block.air()
                                && BlockTraversalData.isWaterPathfindable(below)) {
                            return TerrainType.BREACH;
                        }
                        return TerrainType.BLOCKED;
                    }
                    last = block;
                    lastNamedWater = false;
                }
            }
        }
        return lastNamedWater || BlockTraversalData.isWaterPathfindable(last)
                ? TerrainType.WATER : TerrainType.BLOCKED;
    }

    private TerrainType cachedType(Point point) {
        SearchNode node = node(point);
        if (!node.typeComputed) {
            node.classifiedType = swimType(point);
            node.typeComputed = true;
        }
        return node.classifiedType;
    }

    private static Point startPoint(NavigationRequest request) {
        Point point = request.start();
        return new Vec(
                Math.floor(point.x()
                        + request.boundingBox().relativeStart().x()),
                Math.floor(point.y()
                        + request.boundingBox().relativeStart().y() + 0.5),
                Math.floor(point.z()
                        + request.boundingBox().relativeStart().z()));
    }

    private static int[][] buildSwimDirections() {
        return new int[][]{
                {0, -1, 0}, {0, 1, 0}, {0, 0, -1},
                {0, 0, 1}, {-1, 0, 0}, {1, 0, 0},
                {1, 0, -1}, {1, 0, 1},
                {-1, 0, 1}, {-1, 0, -1}
        };
    }
}
