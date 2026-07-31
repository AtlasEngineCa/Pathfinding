package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;
import ca.atlasengine.pathfinding.internal.JumpArc;
import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;
import net.minestom.server.instance.block.BlockFace;

import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Ballistic platform-to-platform jumps. This is an extension beyond
 * the ordinary ground graph, which emits step, fall, and walk edges.
 */
final class PlatformJumpSupport {
    private final GroundNodeEvaluator evaluator;
    private final PlatformJumpCapabilities capabilities;
    private final BooleanSupplier sampleBudget;

    PlatformJumpSupport(GroundNodeEvaluator evaluator,
                        PlatformJumpCapabilities capabilities) {
        this.evaluator = evaluator;
        this.capabilities = capabilities;
        this.sampleBudget = () -> {
            evaluator.countExamined();
            return !evaluator.interrupted();
        };
    }

    void addNeighbors(SearchNode current, List<SearchNode> result) {
        if (!capabilities.enabled() || evaluator.isAmphibious()
                || current.type == TerrainType.WATER
                || current.type == TerrainType.LAVA
                || !fullSupport(current)) {
            return;
        }
        int maximumDistance = (int) Math.floor(
                capabilities.maxHorizontalDistance());
        int maximumRise = (int) Math.floor(capabilities.maxRise());
        int maximumDrop = (int) Math.floor(capabilities.maxDrop());
        for (GroundNodeEvaluator.Direction direction
                : GroundNodeEvaluator.CARDINAL) {
            // Jump edges are for cells the mob cannot stand in. If the
            // immediate cell has an ordinary ground/step edge, stairs,
            // slabs, carpets, and flat terrain remain normal movement.
            if (!unwalkable(current.x + direction.x(), current.y,
                    current.z + direction.z())) {
                continue;
            }
            for (int distance = 2;
                 distance <= maximumDistance; distance++) {
                if (evaluator.interrupted()) return;
                int precedingX =
                        current.x + direction.x() * (distance - 1);
                int precedingZ =
                        current.z + direction.z() * (distance - 1);
                if (!unwalkable(precedingX, current.y, precedingZ)) {
                    break;
                }
                int x = current.x + direction.x() * distance;
                int z = current.z + direction.z() * distance;
                SearchNode landing = null;
                for (int y = current.y + maximumRise;
                     y >= current.y - maximumDrop; y--) {
                    evaluator.countExamined();
                    if (evaluator.interrupted()) return;
                    TerrainType landingType = evaluator.type(x, y, z);
                    if (!landingType(landingType)) continue;
                    SearchNode candidate = evaluator.accepted(x, y, z);
                    if (candidate.malus < 0 || candidate.hardBlocked
                            || !landingClear(current, candidate)) {
                        continue;
                    }
                    landing = candidate;
                    break;
                }
                if (landing != null && !landing.settled
                        && !result.contains(landing)) {
                    result.add(landing);
                }
            }
        }
    }

    /**
     * True while the cell is not one the mob may stand in, so flying over it
     * is the only way past. Supported and walkable are different things: a
     * cactus, a fence, or a pillar rests on the ground without being
     * anywhere the mob may stand, while ordinary ground it could step onto
     * keeps its walk edge and never becomes a jump.
     *
     * <p>The anchored classifier reports WALKABLE for every anchor whose
     * footprint still overlaps its own platform, so a mob wider or deeper
     * than one cell would never see the gap beside it.</p>
     */
    private boolean unwalkable(int x, int y, int z) {
        TerrainType type = evaluator.type(x, y, z);
        if (type == TerrainType.OPEN
                || evaluator.profile.malus(type) < 0) return true;
        BoundingBox box = evaluator.box;
        return (GroundNodeEvaluator.footprintWidth(box) > 1
                || GroundNodeEvaluator.footprintDepth(box) > 1)
                && !fullSupport(x, y, z);
    }

    private boolean fullSupport(SearchNode node) {
        return fullSupport(node.x, node.y, node.z, false);
    }

    private boolean fullSupport(int nodeX, int nodeY, int nodeZ) {
        return fullSupport(nodeX, nodeY, nodeZ, false);
    }

    /**
     * @param opened whether the follower opens this node's own footprint,
     *               which is the cell range swept below
     */
    private boolean fullSupport(int nodeX, int nodeY, int nodeZ,
                                boolean opened) {
        BoundingBox box = evaluator.box;
        int width = GroundNodeEvaluator.footprintWidth(box);
        int depth = GroundNodeEvaluator.footprintDepth(box);
        for (int x = nodeX; x < nodeX + width; x++) {
            for (int z = nodeZ; z < nodeZ + depth; z++) {
                Block bodyCell = evaluator.blocks.getBlock(
                        x, nodeY, z,
                        Block.Getter.Condition.TYPE);
                if (!bodyCellClear(bodyCell, opened)) {
                    return false;
                }
                Block support = evaluator.blocks.getBlock(
                        x, nodeY - 1, z,
                        Block.Getter.Condition.TYPE);
                var shape = support.collisionShape();
                if (shape.relativeEnd().y() < 1
                        || !shape.isFaceFull(BlockFace.TOP)) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * A cell the mob's body may occupy. An openable block in the footprint
     * the follower opens is left to the swept test rather than rejected here:
     * an open door, trapdoor, or gate is a shape, not air, and the follower
     * sweeps that same shape. Every other cell keeps the strict rule, so a
     * profile that opens nothing reads exactly what it always did.
     */
    private boolean bodyCellClear(Block block, boolean opened) {
        if (block.collisionShape().relativeEnd().y() <= 0) {
            return true;
        }
        return opened && evaluator.profile.canPassDoors()
                && evaluator.profile.blockManipulation().manipulates(
                        BlockTraversalData.openableFamily(block));
    }

    private static boolean landingType(TerrainType type) {
        return switch (type) {
            case WALKABLE, WALKABLE_DOOR, RAIL,
                 FIRE_IN_NEIGHBOR, DAMAGING_IN_NEIGHBOR,
                 WATER_BORDER, DAMAGE_CAUTIOUS,
                 ON_TOP_OF_TRAPDOOR, ON_TOP_OF_POWDER_SNOW -> true;
            default -> false;
        };
    }

    /**
     * Judges one landing against the world the follower will sweep. The
     * follower opens the cells the landing waypoint's box reaches as the
     * first act of executing the jump, so those are read here in the states
     * it opens them into. No other cell is: nothing opens a block the arc
     * merely passes over, and reading one as open would emit an edge every
     * launch refuses.
     */
    private boolean landingClear(SearchNode start, SearchNode landing) {
        Point waypoint = waypoint(landing);
        return fullSupport(landing.x, landing.y, landing.z, true)
                && arcClear(start, landing, waypoint);
    }

    private boolean arcClear(SearchNode start, SearchNode landing, Point to) {
        // The follower sweeps this arc from the entity's live position before
        // it will launch, and a refusal there is unrecoverable. Deciding the
        // edge with the same swept routine is what keeps the two in step.
        return JumpArc.clear(openedLanding(to), evaluator.box, waypoint(start),
                to, capabilities.apexClearance(), sampleBudget);
    }

    private Point waypoint(SearchNode node) {
        BoundingBox box = evaluator.box;
        return new Vec(
                node.x + GroundNodeEvaluator.footprintWidth(box) * 0.5,
                evaluator.waypointY(node.x, node.y, node.z),
                node.z + GroundNodeEvaluator.footprintDepth(box) * 0.5);
    }

    /**
     * The landing footprint in the states {@code BlockManipulator} opens it
     * into, over the same cells and by the same predicate it uses. A profile
     * that opens nothing gets the world itself.
     */
    private Block.Getter openedLanding(Point waypoint) {
        return evaluator.profile.openedFootprintView(
                evaluator.blocks, waypoint, evaluator.box);
    }

    static float arcLength(
            double dx, double dy, double dz, double apexClearance) {
        double horizontal = Math.hypot(dx, dz);
        int samples = Math.max(
                8, (int) Math.ceil(horizontal * 16));
        double length = 0;
        double previousX = 0;
        double previousY = 0;
        double previousZ = 0;
        for (int sample = 1; sample <= samples; sample++) {
            double t = (double) sample / samples;
            double x = dx * t;
            double y = dy * t
                    + 4 * apexClearance * t * (1 - t);
            double z = dz * t;
            double stepX = x - previousX;
            double stepY = y - previousY;
            double stepZ = z - previousZ;
            length += Math.sqrt(
                    stepX * stepX + stepY * stepY + stepZ * stepZ);
            previousX = x;
            previousY = y;
            previousZ = z;
        }
        return (float) length;
    }
}
