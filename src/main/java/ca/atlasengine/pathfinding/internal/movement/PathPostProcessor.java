package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.terrain.BlockTagIndex;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;

import java.util.ArrayList;
import java.util.List;

/**
 * Tick-thread transformations applied to a route before it is followed.
 */
public final class PathPostProcessor {
    /** A processed route and whether it was cut short by sunlight. */
    public record Result(List<PathNode> nodes, boolean truncatedForSun) {
    }

    private PathPostProcessor() {
    }

    public static Result process(Instance instance, NavigationProfile profile,
                                 Point entityPosition, List<PathNode> path) {
        return trimAtFirstSkyExposedNode(instance, profile, entityPosition,
                adjustCauldronNodes(instance, path));
    }

    private static List<PathNode> adjustCauldronNodes(
            Instance instance, List<PathNode> path) {
        if (instance == null || path.isEmpty()) return path;
        List<PathNode> adjusted = new ArrayList<>(path);
        for (int i = 0; i < adjusted.size(); i++) {
            PathNode node = adjusted.get(i);
            Block block = instance.getBlock(
                    node.graphX(), node.graphY(), node.graphZ(),
                    Block.Getter.Condition.TYPE);
            if (!BlockTagIndex.contains("cauldrons", block)) continue;

            int raisedY = node.graphY() + 1;
            adjusted.set(i, new PathNode(
                    node.x(), node.y() + 1, node.z(),
                    PathNode.Movement.STEP_UP,
                    node.graphX(), raisedY, node.graphZ()));
            if (i + 1 < adjusted.size()) {
                PathNode next = adjusted.get(i + 1);
                if (node.graphY() >= next.graphY()) {
                    adjusted.set(i + 1, new PathNode(
                            next.x(), node.y() + 1, next.z(),
                            PathNode.Movement.WALK,
                            next.graphX(), raisedY, next.graphZ()));
                }
            }
        }
        return List.copyOf(adjusted);
    }

    /**
     * Sun-avoiding ground navigation keeps a route only up to the first
     * sky-exposed node, provided the entity began beneath cover.
     */
    private static Result trimAtFirstSkyExposedNode(
            Instance instance, NavigationProfile profile,
            Point entityPosition, List<PathNode> path) {
        if (!profile.avoidSun() || path.isEmpty()
                || profile.mode() != NavigationMode.GROUND) {
            return new Result(path, false);
        }
        if (instance == null || canSeeSky(instance, entityPosition)) {
            return new Result(path, false);
        }
        for (int i = 0; i < path.size(); i++) {
            if (canSeeSky(instance, path.get(i).asVec())) {
                return new Result(List.copyOf(path.subList(0, i)), true);
            }
        }
        return new Result(path, false);
    }

    private static boolean canSeeSky(Instance instance, Point point) {
        var chunk = instance.getChunkAt(point);
        if (chunk == null) return false;
        int highestMotionBlocker = chunk.motionBlockingHeightmap().getHeight(
                point.blockX() & 15, point.blockZ() & 15);
        return highestMotionBlocker < point.blockY();
    }
}
