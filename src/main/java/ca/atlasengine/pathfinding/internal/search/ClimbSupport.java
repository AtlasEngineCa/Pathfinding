package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.influence.InfluenceResult;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;

import java.util.List;

/**
 * Planned climbing over tagged climbable blocks. This is an extension beyond
 * the ordinary ground graph: baseline movement leaves ladders to entity physics
 * and never emits vertical edges for them.
 */
final class ClimbSupport {
    private final GroundNodeEvaluator evaluator;
    private final ClimbableCapabilities capabilities;

    ClimbSupport(GroundNodeEvaluator evaluator,
                 ClimbableCapabilities capabilities) {
        this.evaluator = evaluator;
        this.capabilities = capabilities;
    }

    boolean climbableAt(int x, int y, int z) {
        return BlockTraversalData.isClimbableAt(evaluator.blocks, x, y, z);
    }

    boolean onClimbable(SearchNode node) {
        return capabilities.enabled()
                && climbableAt(node.x, node.y, node.z);
    }

    SearchNode adjustCardinal(SearchNode neighbor, SearchNode current,
                              GroundNodeEvaluator.Direction direction,
                              boolean currentOnClimbable) {
        if (!capabilities.enabled()) return neighbor;
        // A ladder-face anchor cannot execute an ordinary diagonal
        // step through the backing block onto its top. Climb clear of
        // the top edge first, then leave horizontally.
        if (currentOnClimbable && neighbor != null
                && neighbor.y > current.y) {
            neighbor = null;
        }
        int x = current.x + direction.x();
        int z = current.z + direction.z();
        if (climbableAt(x, current.y, z)
                || climbableAt(x, current.y - 1, z)) {
            SearchNode entry = entryNode(x, current.y, z);
            if (entry != null) return entry;
        }
        return neighbor;
    }

    void addNeighbors(SearchNode current, List<SearchNode> result) {
        if (!capabilities.enabled()) return;
        if (capabilities.allowAscending()
                && connected(current.x, current.y, current.z, 1)) {
            evaluator.countExamined();
            SearchNode up = entryNode(
                    current.x, current.y + 1, current.z);
            if (evaluator.validCardinal(up, current)
                    && !result.contains(up)) {
                result.add(up);
            }
        }
        if (capabilities.allowDescending()
                && connected(current.x, current.y, current.z, -1)) {
            evaluator.countExamined();
            SearchNode down = entryNode(
                    current.x, current.y - 1, current.z);
            if (evaluator.validCardinal(down, current)
                    && !result.contains(down)) {
                result.add(down);
            }
        }
    }

    private boolean connected(int x, int y, int z, int dy) {
        boolean here = climbableAt(x, y, z);
        boolean next = climbableAt(x, y + dy, z);
        if (!here && !next) return false;
        // One clear node beyond the end of a column permits a safe exit.
        // Do not use the same allowance to bridge an internal missing
        // cell between two tagged climbables.
        return !here || next || !climbableAt(x, y + dy * 2, z);
    }

    private SearchNode entryNode(int x, int y, int z) {
        BoundingBox box = evaluator.box;
        Point position = waypoint(x, y, z);
        if (occupiedWhileClimbing(position)) return null;
        SearchNode node = evaluator.node(x, y, z);
        node.type = TerrainType.OPEN;
        double malus = 0;
        for (NavigationInfluence influence : evaluator.influences) {
            InfluenceResult result = influence.evaluate(
                    evaluator.blocks, position, box, evaluator.control);
            if (result.blocked()) {
                node.hardBlocked = true;
                node.malus = -1;
                return null;
            }
            malus += result.costDelta();
        }
        node.malus = Math.max(node.malus, Math.max(0, malus));
        return node.malus >= 0 && !node.hardBlocked ? node : null;
    }

    /**
     * The waypoint the evaluator's {@code result()} emits for this node.
     * Clearing any other point would clear one the follower never stands at.
     */
    private Point waypoint(int x, int y, int z) {
        BoundingBox box = evaluator.box;
        return new Vec(
                waypointX(x, y, z,
                        GroundNodeEvaluator.footprintWidth(box) * 0.5),
                evaluator.waypointY(x, y, z),
                waypointZ(x, y, z,
                        GroundNodeEvaluator.footprintDepth(box) * 0.5));
    }

    /**
     * Judges one climb node against the world the follower will sweep at it.
     * The follower opens the cells its box reaches as it drives to the
     * waypoint, so those are read in the states it opens them into. No other
     * cell is: nothing opens a block the mob never stands over, and reading
     * one as air emits a node the follower's identical test then refuses.
     */
    private boolean occupiedWhileClimbing(Point position) {
        BoundingBox box = evaluator.box;
        Block.Getter blocks = evaluator.profile.openedFootprintView(
                evaluator.blocks, position, box);
        var iterator = box.getBlocks(position);
        while (iterator.hasNext()) {
            var mutable = iterator.next();
            Block block = blocks.getBlock(mutable.blockX(),
                    mutable.blockY(), mutable.blockZ(),
                    Block.Getter.Condition.TYPE);
            if (BlockTraversalData.isClimbable(block)) continue;
            Point blockPosition = new Vec(mutable.blockX(),
                    mutable.blockY(), mutable.blockZ());
            if (block.collisionShape().intersectBox(
                    position.sub(blockPosition), box)) return true;
        }
        return false;
    }

    boolean anchoredAt(int x, int y, int z) {
        return capabilities.enabled()
                && (climbableAt(x, y, z) || climbableAt(x, y - 1, z));
    }

    boolean isClimbMove(PathNode previous, PathNode node) {
        if (previous == null || !capabilities.enabled()) return false;
        if (previous.graphX() == node.graphX()
                && previous.graphZ() == node.graphZ()
                && previous.graphY() != node.graphY()
                && (at(previous) || at(node))) {
            return true;
        }
        if (previous.graphY() == node.graphY()
                && (anchored(previous) || anchored(node))) {
            return true;
        }
        return node.graphY() <= previous.graphY()
                && Math.abs(node.graphX() - previous.graphX()) <= 1
                && Math.abs(node.graphZ() - previous.graphZ()) <= 1
                && anchored(previous);
    }

    private boolean at(PathNode node) {
        return climbableAt(node.graphX(), node.graphY(), node.graphZ());
    }

    private boolean anchored(PathNode node) {
        return climbableAt(node.graphX(), node.graphY(), node.graphZ())
                || climbableAt(node.graphX(),
                node.graphY() - 1, node.graphZ());
    }

    double waypointX(int x, int y, int z, double defaultOffset) {
        Block block = evaluator.blocks.getBlock(
                x, y, z, Block.Getter.Condition.TYPE);
        if (!block.compare(Block.LADDER)) {
            return x + defaultOffset;
        }
        // Leave a small physics margin outside the climb face. Merely
        // touching the voxel plane is treated as intersecting by swept
        // Minestom collision on the following vertical tick.
        double half = anchorOffset(defaultOffset, evaluator.box.width());
        String facing = block.getProperty("facing");
        return switch (facing == null ? "north" : facing) {
            case "west" -> x - half;
            case "east" -> x + 1 + half;
            default -> x + defaultOffset;
        };
    }

    double waypointZ(int x, int y, int z, double defaultOffset) {
        Block block = evaluator.blocks.getBlock(
                x, y, z, Block.Getter.Condition.TYPE);
        if (!block.compare(Block.LADDER)) {
            return z + defaultOffset;
        }
        double half = anchorOffset(defaultOffset, evaluator.box.depth());
        String facing = block.getProperty("facing");
        return switch (facing == null ? "north" : facing) {
            case "north" -> z - half;
            case "south" -> z + 1 + half;
            default -> z + defaultOffset;
        };
    }

    /**
     * Distance from the ladder cell to an anchored waypoint's center. A
     * planned climb hangs the mob's whole grid footprint outside that cell,
     * at the offset the evaluator emits on every unanchored axis, so the box
     * the probe clears and the box the follower drives are the same box.
     * Without planned climbing nothing probes and the offset stays put.
     */
    private double anchorOffset(double defaultOffset, double size) {
        return (capabilities.enabled() ? defaultOffset : size * 0.5) + 0.01;
    }
}
