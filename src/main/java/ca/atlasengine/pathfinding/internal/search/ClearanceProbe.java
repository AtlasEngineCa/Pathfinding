package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;

/**
 * Asks whether a box fits. Every question here is about geometry against the
 * world: the probe decides no traversal costs and does not mutate graph state.
 * It retains the evaluator for shared world data and interruption checks while
 * keeping clearance decisions separate from edge selection.
 *
 * <p>Two distinct questions live here. A <em>static</em> fit asks whether the
 * box overlaps anything at a resting position. A <em>swept</em> fit asks
 * whether the corridor between two cells is clear, which is a different
 * question whenever a block's shape depends on which way you cross it.</p>
 */
final class ClearanceProbe {
    private final GroundNodeEvaluator evaluator;
    private final Block.Getter blocks;
    private final Block.Getter collisionBlocks;
    private final BoundingBox box;
    private final Point startPosition;
    /** Whether any openable is ever read in its swung state. */
    private final boolean opensBlocks;

    ClearanceProbe(GroundNodeEvaluator evaluator, boolean opensBlocks) {
        this.evaluator = evaluator;
        this.blocks = evaluator.blocks;
        this.collisionBlocks = evaluator.collisionBlocks;
        this.box = evaluator.box;
        this.startPosition = evaluator.startPosition;
        this.opensBlocks = opensBlocks;
    }

    /**
     * Whether a shape stands inside the box at this position. Reads the
     * clearance view, so an openable this profile swings is measured in the
     * state the follower will leave it in.
     */
    boolean occupied(Point position, BoundingBox testedBox) {
        var iterator = testedBox.getBlocks(position);
        int scanned = 0;
        while (iterator.hasNext()) {
            if (evaluator.control.interruptible() && (scanned++ & 63) == 0
                    && evaluator.interrupted()) return true;
            var mutable = iterator.next();
            int blockX = mutable.blockX();
            int blockY = mutable.blockY();
            int blockZ = mutable.blockZ();
            Block block = collisionBlocks.getBlock(
                    blockX, blockY, blockZ, Block.Getter.Condition.TYPE);
            if (block.collisionShape().intersectBox(
                    position.sub(blockX, blockY, blockZ), testedBox)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Whether the entity could travel from where it stands to this node
     * without striking anything, sampled at box-sized intervals. Used only
     * where the source cell holds a partial barrier, since that is the case a
     * per-cell type cannot answer.
     */
    boolean reachableWithoutCollision(SearchNode target) {
        double dx = target.x - startPosition.x() + box.width() / 2;
        double dy = target.y - startPosition.y() + box.height() / 2;
        double dz = target.z - startPosition.z() + box.depth() / 2;
        double length = Math.sqrt(dx * dx + dy * dy + dz * dz);
        double averageSize = (box.width() + box.height() + box.depth()) / 3;
        int steps = (int) Math.ceil(length / averageSize);
        if (steps <= 0) return true;
        Vec delta = new Vec(dx / steps, dy / steps, dz / steps);
        Point position = startPosition;
        for (int i = 0; i < steps; i++) {
            position = position.add(delta);
            if (occupied(position, box)) return false;
        }
        return true;
    }

    /**
     * Whether the panel an openable swings to stands in the corridor this move
     * sweeps. A node type says what a cell is, never which way you may enter
     * it, so the direction lives on the edge instead: an open trapdoor walls
     * off the axis it swung across while leaving the other one clear, and only
     * a box narrow enough to pass beside it may take that one.
     *
     * <p>An entity whose own resting box already overlaps a panel is not made
     * worse off by moving, so it keeps every edge it had.</p>
     */
    boolean swungPanelBlocks(SearchNode from, SearchNode to) {
        if (!opensBlocks || from == to) return false;
        return sweepHitsPanel(from, to) && !sweepHitsPanel(from, from);
    }

    private boolean sweepHitsPanel(SearchNode from, SearchNode to) {
        double halfWidth = box.width() / 2;
        double halfDepth = box.depth() / 2;
        double centerX = GroundNodeEvaluator.footprintWidth(box) * 0.5;
        double centerZ = GroundNodeEvaluator.footprintDepth(box) * 0.5;
        double minX = Math.min(from.x, to.x) + centerX - halfWidth;
        double maxX = Math.max(from.x, to.x) + centerX + halfWidth;
        double minZ = Math.min(from.z, to.z) + centerZ - halfDepth;
        double maxZ = Math.max(from.z, to.z) + centerZ + halfDepth;
        double fromFloor = evaluator.floorLevel(from.x, from.y, from.z);
        double toFloor = evaluator.floorLevel(to.x, to.y, to.z);
        double minY = Math.min(fromFloor, toFloor);
        double maxY = Math.max(fromFloor, toFloor) + box.height();
        Point position = new Vec(
                (minX + maxX) / 2, minY, (minZ + maxZ) / 2);
        BoundingBox swept = new BoundingBox(
                maxX - minX, maxY - minY, maxZ - minZ);
        var iterator = swept.getBlocks(position);
        while (iterator.hasNext()) {
            var mutable = iterator.next();
            int blockX = mutable.blockX();
            int blockY = mutable.blockY();
            int blockZ = mutable.blockZ();
            Block block = blocks.getBlock(
                    blockX, blockY, blockZ, Block.Getter.Condition.TYPE);
            if (block.air()) continue;
            if (BlockTraversalData.obstructsWhenOpen(
                    block, evaluator.profile.blockManipulation(),
                    position.sub(blockX, blockY, blockZ), swept)) return true;
        }
        return false;
    }

    /**
     * Whether a cell's barrier occupies only part of its volume. These are the
     * types a mob may be standing inside rather than against, so leaving one
     * needs the swept check above rather than a cell lookup.
     */
    static boolean partialBarrier(TerrainType type) {
        return type == TerrainType.FENCE
                || type == TerrainType.DOOR_WOOD_CLOSED
                || type == TerrainType.DOOR_IRON_CLOSED;
    }
}
