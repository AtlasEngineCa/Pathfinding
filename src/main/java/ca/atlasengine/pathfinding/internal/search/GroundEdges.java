package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.profile.GroundCapabilities;
import ca.atlasengine.pathfinding.terrain.TerrainCosts;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Vec;

/**
 * Decides which cell an entity actually arrives in when it moves one step in a
 * given direction — which is rarely the cell that was asked for.
 *
 * <p>Walking at a cell is a request, not an outcome. The cell may be enterable
 * as-is, or reachable only by stepping up onto it, or empty so the entity
 * falls through to whatever is below, or water it sinks through, or a barrier
 * it stops against. Each case resolves to a different node, and the order they
 * are tried in is the movement policy: a stable surface wins outright, a step
 * up is preferred to a fall, and settling is the last resort.</p>
 */
final class GroundEdges {
    /**
     * Default jump reach when a profile does not raise it. A step is a legal
     * move up to the taller of this and the profile's own step height.
     */
    private static final double DEFAULT_JUMP_HEIGHT = 1.125;

    /**
     * How far a clearance probe is held off the surfaces that define it. The
     * jump-corridor box runs from the takeoff floor to the landing head
     * height; without this margin it would register the very floor it stands
     * on and the ceiling it is measured against. The ceiling gets twice the
     * margin because the floor below it has already consumed one.
     */
    private static final double SURFACE_EPSILON = 0.001;

    private final GroundNodeEvaluator evaluator;
    private final ClearanceProbe clearance;
    private final GroundCapabilities capabilities;
    private final BoundingBox box;

    GroundEdges(GroundNodeEvaluator evaluator, ClearanceProbe clearance,
                GroundCapabilities capabilities) {
        this.evaluator = evaluator;
        this.clearance = clearance;
        this.capabilities = capabilities;
        this.box = evaluator.box;
    }

    /**
     * The node reached by stepping toward {@code (x, y, z)}, or {@code null}
     * if the move is not available at all.
     *
     * @param jumpSize    remaining upward steps this move may still spend
     * @param sourceFloor collision-floor height of the cell being left
     */
    SearchNode resolve(int x, int y, int z, int jumpSize, double sourceFloor,
                       GroundNodeEvaluator.Direction direction,
                       TerrainType sourceType) {
        if (evaluator.floorLevel(x, y, z) - sourceFloor > jumpHeight()) {
            return null;
        }
        TerrainType targetType = evaluator.type(x, y, z);
        SearchNode direct = directEntry(x, y, z, targetType, sourceType);

        if (standableSurface(targetType)) return direct;
        if (worthStepUp(targetType, direct, jumpSize)) {
            return stepUp(x, y, z, jumpSize, sourceFloor, direction, sourceType);
        }
        return settle(x, y, z, targetType, direct);
    }

    /** The cell as-is, if the profile may occupy it and can get there. */
    private SearchNode directEntry(int x, int y, int z, TerrainType targetType,
                                   TerrainType sourceType) {
        if (evaluator.profile.malus(targetType) < 0) return null;
        SearchNode candidate = evaluator.accepted(x, y, z);
        // Leaving a partial barrier is the one case a per-cell type cannot
        // answer, because the barrier is inside the cell being left.
        if (ClearanceProbe.partialBarrier(sourceType) && candidate.malus >= 0
                && !clearance.reachableWithoutCollision(candidate)) return null;
        return candidate;
    }

    private boolean standableSurface(TerrainType type) {
        return type == TerrainType.WALKABLE
                || (evaluator.isAmphibious() && type == TerrainType.WATER);
    }

    /**
     * Whether to spend an upward step here. Only worth trying when the cell is
     * not already enterable and the obstruction is one a mob can actually
     * mount — a fence is climbable only by profiles that say so, and the rest
     * are surfaces that give way underfoot rather than support a step.
     */
    private boolean worthStepUp(
            TerrainType type, SearchNode direct, int remainingSteps) {
        if (remainingSteps <= 0 || direct != null && direct.malus >= 0) {
            return false;
        }
        return (type != TerrainType.FENCE
                    || evaluator.profile.canWalkOverFences())
                && type != TerrainType.UNPASSABLE_RAIL
                && type != TerrainType.TRAPDOOR
                && type != TerrainType.POWDER_SNOW;
    }

    /** Where the entity ends up when it neither enters nor steps up. */
    private SearchNode settle(
            int x, int y, int z, TerrainType type, SearchNode direct) {
        if (!evaluator.isAmphibious() && type == TerrainType.WATER
                && !evaluator.profile.canFloat()) {
            return sinkThroughWater(x, y, z, direct);
        }
        if (type == TerrainType.OPEN) return fallToGround(x, y, z);
        if (ClearanceProbe.partialBarrier(type) && direct == null) {
            return barrier(x, y, z, type);
        }
        return direct;
    }

    private double jumpHeight() {
        return Math.max(DEFAULT_JUMP_HEIGHT, capabilities.maxStepHeight());
    }

    /**
     * Recurses one cell higher with one less step to spend, then verifies the
     * entity would fit in the corridor it has to travel through to get there.
     * The corridor check is skipped for entities at least a block wide, which
     * cannot thread a gap narrower than themselves anyway.
     */
    private SearchNode stepUp(int x, int y, int z, int jumpSize,
                              double sourceFloor,
                              GroundNodeEvaluator.Direction direction,
                              TerrainType sourceType) {
        SearchNode above = resolve(x, y + 1, z, jumpSize - 1,
                sourceFloor, direction, sourceType);
        if (above != null && evaluator.footprintFloorLevel(
                above.x, above.y, above.z) - sourceFloor > jumpHeight()) {
            return null;
        }
        if (above == null || box.width() >= 1
                || (above.type != TerrainType.OPEN
                && above.type != TerrainType.WALKABLE)) return above;

        double centerX = x - direction.x() + 0.5;
        double centerZ = z - direction.z() + 0.5;
        double floor = evaluator.floorLevel(
                (int) Math.floor(centerX), y + 1, (int) Math.floor(centerZ));
        double ceiling = box.height()
                + evaluator.floorLevel(above.x, above.y, above.z);
        double from = floor + SURFACE_EPSILON;
        double to = ceiling - 2 * SURFACE_EPSILON;
        if (to <= from) return above;

        return clearance.occupied(new Vec(centerX, from, centerZ),
                new BoundingBox(box.width(), to - from, box.width()))
                ? null : above;
    }

    /** A non-floating entity descends until the water column ends. */
    private SearchNode sinkThroughWater(
            int x, int y, int z, SearchNode initialWater) {
        SearchNode best = initialWater;
        for (int currentY = y - 1; currentY > evaluator.minY; currentY--) {
            if (evaluator.type(x, currentY, z) != TerrainType.WATER) return best;
            best = evaluator.accepted(x, currentY, z);
        }
        return best;
    }

    /**
     * Falls through empty cells to the first supporting one. A drop past the
     * profile's fall limit yields a blocked node rather than nothing, so the
     * cell is recorded as refused instead of being probed again by every
     * neighbouring expansion.
     */
    private SearchNode fallToGround(int x, int y, int z) {
        for (int currentY = y - 1; currentY >= evaluator.minY; currentY--) {
            if (y - currentY > (int) capabilities.maxFallDistance()) {
                return blocked(x, currentY, z);
            }
            TerrainType type = evaluator.type(x, currentY, z);
            if (type == TerrainType.OPEN) continue;
            return evaluator.profile.malus(type) >= 0
                    ? evaluator.accepted(x, currentY, z)
                    : blocked(x, currentY, z);
        }
        return blocked(x, y, z);
    }

    private SearchNode blocked(int x, int y, int z) {
        SearchNode node = evaluator.node(x, y, z);
        node.type = TerrainType.BLOCKED;
        node.malus = -1;
        return node;
    }

    /**
     * A partial barrier the entity stops against. Pre-settled so the search
     * records the cell without ever expanding from it.
     */
    private SearchNode barrier(int x, int y, int z, TerrainType type) {
        SearchNode node = evaluator.node(x, y, z);
        node.type = type;
        node.malus = TerrainCosts.baseline(type);
        node.settled = true;
        return node;
    }
}
