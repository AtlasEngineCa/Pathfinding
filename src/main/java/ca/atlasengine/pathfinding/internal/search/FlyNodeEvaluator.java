package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.search.SearchControl;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainClassification;
import ca.atlasengine.pathfinding.terrain.TerrainClassifier;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;

import java.util.EnumSet;
import java.util.List;

/**
 * Volume evaluator for free-flight movement.
 */
public final class FlyNodeEvaluator extends SpatialNodeEvaluator {
    private static final int[][] FLY_DIRECTIONS = buildFlyDirections();
    private final List<SearchNode> neighbors = new java.util.ArrayList<>(26);

    public FlyNodeEvaluator(NavigationRequest request, SearchControl control) {
        super(request, control);
    }

    FlyNodeEvaluator(NavigationRequest request) {
        this(request, SearchControl.NONE);
    }

    @Override
    SearchNode getStart() {
        Point start = startPoint();
        SearchNode node = node(start);
        TerrainType type = cachedType(start);
        node.type = type;
        node.malus = (float) mobProfile.malus(type);
        return node;
    }

    @Override
    int lastExaminedNeighbors() {
        return FLY_DIRECTIONS.length;
    }

    @Override
    PathNode.Movement movement(SearchNode node) {
        return PathNode.Movement.FLY;
    }

    @Override
    SearchNode accepted(Point point) {
        TerrainType type = cachedType(point);
        float base = (float) mobProfile.malus(type);
        SearchNode node = node(point);
        if (base < 0) {
            node.malus = -1;
            return null;
        }
        node.type = type;
        node.malus = Math.max(node.malus, base);
        if (type == TerrainType.WALKABLE) node.malus = (float) (node.malus + 1);
        return node;
    }

    @Override
    List<SearchNode> getNeighbors(SearchNode current) {
        neighbors.clear();
        for (int[] direction : FLY_DIRECTIONS) {
            Point point = point(current).add(
                    direction[0], direction[1], direction[2]);
            SearchNode candidate = accepted(point);
            if (candidate == null || candidate.settled
                    || !flyPrerequisites(current, direction)) {
                continue;
            }
            neighbors.add(candidate);
        }
        return neighbors;
    }

    private boolean flyPrerequisites(SearchNode current, int[] direction) {
        int[] axes = new int[3];
        int changed = 0;
        for (int axis = 0; axis < 3; axis++) {
            if (direction[axis] != 0) axes[changed++] = axis;
        }
        for (int mask = 1; mask < (1 << changed) - 1; mask++) {
            int[] offset = new int[3];
            for (int bit = 0; bit < changed; bit++) {
                if ((mask & 1 << bit) != 0) {
                    int axis = axes[bit];
                    offset[axis] = direction[axis];
                }
            }
            Point point = point(current).add(
                    offset[0], offset[1], offset[2]);
            SearchNode prerequisite = nodes.get(
                    point.blockX(), point.blockY(), point.blockZ());
            if (prerequisite == null || prerequisite.malus < 0
                    || combinedInfluence(point).blocked()) return false;
        }
        return true;
    }

    double flyMalus(Point point) {
        TerrainType type = flyVolumeType(point);
        double malus = mobProfile.malus(type);
        return type == TerrainType.WALKABLE ? malus + 1 : malus;
    }

    private TerrainType cachedType(Point point) {
        SearchNode node = node(point);
        if (!node.typeComputed) {
            node.classifiedType = flyVolumeType(point);
            node.typeComputed = true;
        }
        return node.classifiedType;
    }

    TerrainType flyVolumeType(Point point) {
        int minX = point.blockX();
        int minY = point.blockY();
        int minZ = point.blockZ();
        int sizeX = Math.max(1,
                (int) Math.floor(request.boundingBox().width() + 1));
        int sizeY = Math.max(1,
                (int) Math.floor(request.boundingBox().height() + 1));
        int sizeZ = Math.max(1,
                (int) Math.floor(request.boundingBox().depth() + 1));
        MobTraversalProfile profile = mobProfile;
        TerrainClassification classification = profile.classification();
        EnumSet<TerrainType> types = EnumSet.noneOf(TerrainType.class);
        int scanned = 0;
        for (int x = minX; x < minX + sizeX; x++) {
            for (int y = minY; y < minY + sizeY; y++) {
                for (int z = minZ; z < minZ + sizeZ; z++) {
                    if (control.interruptible() && (scanned++ & 63) == 0
                            && (control.cancelled() || control.timedOut())) {
                        return TerrainType.BLOCKED;
                    }
                    types.add(TerrainClassifier.transform(
                            flyTypeAt(x, y, z, classification), profile));
                }
            }
        }
        // The volume search keeps its single-type shortcut: unlike the walk
        // evaluator it never falls back to the blocked-node ranking below.
        if (types.size() == 1) return types.iterator().next();
        // Compare the untransformed anchor cell before volume aggregation.
        return TerrainClassifier.select(types,
                flyTypeAt(minX, minY, minZ, classification), profile,
                request.boundingBox());
    }

    private TerrainType flyTypeAt(
            int x, int y, int z, TerrainClassification classification) {
        TerrainType type = TerrainClassifier.raw(request.blocks().getBlock(
                x, y, z, Block.Getter.Condition.TYPE), classification);
        if (type == TerrainType.OPEN
                && y >= request.entityState().minBuildHeight() + 1) {
            TerrainType below = TerrainClassifier.raw(request.blocks().getBlock(
                    x, y - 1, z, Block.Getter.Condition.TYPE), classification);
            if (below == TerrainType.FIRE || below == TerrainType.LAVA) {
                type = TerrainType.FIRE;
            } else if (below == TerrainType.DAMAGING) {
                type = TerrainType.DAMAGING;
            } else if (below == TerrainType.COCOA) {
                type = TerrainType.COCOA;
            } else if (below == TerrainType.FENCE) {
                boolean belowIsMobPosition = x == request.start().blockX()
                        && y - 1 == request.start().blockY()
                        && z == request.start().blockZ();
                if (!belowIsMobPosition) type = TerrainType.FENCE;
            } else {
                type = below != TerrainType.WALKABLE
                        && below != TerrainType.OPEN
                        && below != TerrainType.WATER
                        ? TerrainType.WALKABLE : TerrainType.OPEN;
            }
        }
        if (type == TerrainType.WALKABLE || type == TerrainType.OPEN) {
            type = TerrainClassifier.neighborType(
                    request.blocks(), x, y, z, type, classification);
        }
        return type;
    }

    private Point startPoint() {
        Point point = request.start();
        int y = (int) Math.floor(point.y() + 0.5);
        if (mobProfile.canFloat()
                && request.entityState().inWater()) {
            y = point.blockY();
            while (y <= request.entityState().maxBuildHeight()
                    && request.blocks().getBlock(
                    point.blockX(), y, point.blockZ(),
                    Block.Getter.Condition.TYPE).compare(Block.WATER)) {
                y++;
            }
        }
        Point nominal = new Vec(point.blockX(), y, point.blockZ());
        if (flyMalus(nominal) >= 0) return nominal;

        BoundingBox box = request.boundingBox();
        double averageSize = (box.width() + box.height() + box.depth()) / 3;
        if (averageSize >= 1) {
            double minX = point.x() + box.relativeStart().x();
            double maxX = point.x() + box.relativeEnd().x();
            double minZ = point.z() + box.relativeStart().z();
            double maxZ = point.z() + box.relativeEnd().z();
            int candidateY = point.blockY();
            Point[] candidates = {
                    new Vec(Math.floor(minX), candidateY, Math.floor(minZ)),
                    new Vec(Math.floor(minX), candidateY, Math.floor(maxZ)),
                    new Vec(Math.floor(maxX), candidateY, Math.floor(minZ)),
                    new Vec(Math.floor(maxX), candidateY, Math.floor(maxZ))
            };
            for (Point candidate : candidates) {
                if (flyMalus(candidate) >= 0) return candidate;
            }
        } else {
            for (Point candidate :
                    request.entityState().pathfindingStartCandidates()) {
                if (flyMalus(candidate) >= 0) return candidate;
            }
        }
        return nominal;
    }

    private static int[][] buildFlyDirections() {
        return new int[][]{
                {0, 0, 1}, {-1, 0, 0}, {1, 0, 0},
                {0, 0, -1}, {0, 1, 0}, {0, -1, 0},
                {0, 1, 1}, {-1, 1, 0}, {1, 1, 0},
                {0, 1, -1},
                {0, -1, 1}, {-1, -1, 0}, {1, -1, 0},
                {0, -1, -1}, {1, 0, -1}, {1, 0, 1},
                {-1, 0, -1}, {-1, 0, 1},
                {1, 1, -1}, {1, 1, 1}, {-1, 1, -1},
                {-1, 1, 1}, {1, -1, -1}, {1, -1, 1},
                {-1, -1, -1}, {-1, -1, 1}
        };
    }
}
