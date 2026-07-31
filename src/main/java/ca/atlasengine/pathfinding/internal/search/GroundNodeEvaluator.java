package ca.atlasengine.pathfinding.internal.search;

import ca.atlasengine.pathfinding.search.EntityTraversalState;
import ca.atlasengine.pathfinding.profile.GroundCapabilities;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.result.PathStatus;
import ca.atlasengine.pathfinding.search.SearchControl;
import ca.atlasengine.pathfinding.influence.InfluenceResult;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainClassifier;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;

import java.util.ArrayList;
import java.util.List;

/**
 * Integer-grid evaluator for collision-aware ground movement.
 *
 * <p>This class owns the cell store and the neighbour scan: which directions
 * are tried, and which of the resulting candidates survive as edges. Each
 * further concern is a collaborator, because each answers a question that can
 * be asked — and tested — without the others:</p>
 *
 * <ul>
 *   <li>{@link StartCellResolver} — where a search begins.</li>
 *   <li>{@link ClearanceProbe} — whether a box fits, statically or swept.</li>
 *   <li>{@link GroundEdges} — which cell a step actually arrives in.</li>
 *   <li>{@link RouteAssembler} — the settled chain as follower waypoints.</li>
 *   <li>{@link ClimbSupport}, {@link PlatformJumpSupport} — opt-in edges.</li>
 * </ul>
 *
 * <p>Collision-floor heights are deliberately not graph coordinates: they are
 * used only when deciding whether a step is legal and when projecting a node
 * to a movement-control waypoint.</p>
 */
class GroundNodeEvaluator extends NodeEvaluator {
    static final Direction[] CARDINAL = {
            new Direction(0, -1), new Direction(1, 0),
            new Direction(0, 1), new Direction(-1, 0)
    };

    final Block.Getter blocks;
    /**
     * Clearance view of {@link #blocks}. Terrain classification keeps reading
     * the raw states, because an openable block is only routable while its
     * own state still says which family it belongs to.
     */
    final Block.Getter collisionBlocks;
    final Point startPosition;
    final BoundingBox box;
    final MobTraversalProfile profile;
    final GroundCapabilities capabilities;
    final List<NavigationInfluence> influences;
    final EntityTraversalState entityState;
    final SearchControl control;
    final int minY;

    private final TerrainClassifier classifier = new TerrainClassifier();
    private final CoordinateNodeMap nodes = new CoordinateNodeMap();
    private final List<SearchNode> neighbors = new ArrayList<>(16);
    private final SearchNode[] cardinalNeighbors =
            new SearchNode[CARDINAL.length];

    private final ClearanceProbe clearance;
    private final StartCellResolver startCells;
    private final GroundEdges edges;
    private final RouteAssembler routes;
    private final ClimbSupport climb;
    private final PlatformJumpSupport platformJump;

    private int lastExaminedNeighbors;

    GroundNodeEvaluator(Block.Getter blocks, Point startPosition,
                        BoundingBox box, MobTraversalProfile profile,
                        List<NavigationInfluence> influences,
                        EntityTraversalState entityState,
                        GroundSearchSpec spec, SearchControl control) {
        this.blocks = blocks;
        this.profile = profile.withMemoizedClassification();
        this.collisionBlocks = this.profile.collisionView(blocks);
        this.startPosition = startPosition;
        this.box = box;
        this.capabilities = spec.capabilities();
        this.influences = List.copyOf(influences);
        this.entityState = entityState;
        this.minY = entityState.minBuildHeight();
        this.control = control;

        boolean opensBlocks = this.profile.canPassDoors()
                && this.profile.blockManipulation().enabled();
        this.clearance = new ClearanceProbe(this, opensBlocks);
        this.startCells = new StartCellResolver(this);
        this.edges = new GroundEdges(this, clearance, capabilities);
        this.climb = new ClimbSupport(this, capabilities.climbables());
        this.platformJump = new PlatformJumpSupport(
                this, capabilities.platformJump());
        this.routes = new RouteAssembler(this, climb);
    }

    boolean isAmphibious() {
        return false;
    }

    @Override
    SearchNode getStart() {
        return startCells.resolve();
    }

    static boolean isWater(Block block) {
        return BlockTraversalData.hasWaterFluid(block);
    }

    @Override
    int lastExaminedNeighbors() {
        return lastExaminedNeighbors;
    }

    void countExamined() {
        lastExaminedNeighbors++;
    }

    @Override
    float edgeCost(SearchNode from, SearchNode to) {
        float x = to.x - from.x;
        float z = to.z - from.z;
        if ((Math.abs(x) > 1 || Math.abs(z) > 1)
                && capabilities.platformJump().enabled()) {
            return PlatformJumpSupport.arcLength(x, to.y - from.y, z,
                    capabilities.platformJump().apexClearance());
        }
        if (capabilities.horizontalEdgeCost()) {
            return (float) Math.sqrt(x * x + z * z);
        }
        float y = to.y - from.y;
        if (x == 0 && z == 0 && y != 0
                && capabilities.climbables().enabled()) {
            return (float) (Math.abs(y)
                    * capabilities.climbables().verticalCostMultiplier());
        }
        return (float) Math.sqrt(x * x + y * y + z * z);
    }

    @Override
    boolean exceedsPathLength(
            SearchNode start, SearchNode current, double maxPathLength) {
        return distance(start, current) >= maxPathLength
                || current.routeLength >= maxPathLength;
    }

    @Override
    List<SearchNode> getNeighbors(SearchNode current) {
        lastExaminedNeighbors = capabilities.allowDiagonal() ? 8 : 4;
        StepContext step = stepContext(current);
        neighbors.clear();
        scanSides(current, step, cardinalNeighbors, neighbors);
        if (capabilities.allowDiagonal()) {
            scanCorners(current, step, cardinalNeighbors, neighbors);
        }
        addVerticalNeighbors(current, neighbors, step.height(), step.floor());
        climb.addNeighbors(current, neighbors);
        platformJump.addNeighbors(current, neighbors);
        return neighbors;
    }

    /**
     * How much upward reach this expansion has. Honey holds a mob down, and a
     * blocked cell overhead leaves nowhere to rise into, so either grounds the
     * step budget at zero.
     */
    private StepContext stepContext(SearchNode current) {
        boolean headRoom = profile.malus(
                type(current.x, current.y + 1, current.z)) >= 0;
        int height = headRoom && current.type != TerrainType.STICKY_HONEY
                ? (int) Math.floor(Math.max(
                        1, capabilities.maxStepHeight())) : 0;
        return new StepContext(
                height, floorLevel(current.x, current.y, current.z),
                climb.onClimbable(current));
    }

    private void scanSides(
            SearchNode current, StepContext step, SearchNode[] sides,
            List<SearchNode> output) {
        for (int index = 0; index < CARDINAL.length; index++) {
            Direction heading = CARDINAL[index];
            SearchNode candidate = findAccepted(
                    current.x + heading.x(), current.y,
                    current.z + heading.z(), step.height(), step.floor(),
                    heading, current.type);
            candidate = climb.adjustCardinal(
                    candidate, current, heading, step.onClimbable());
            sides[index] = candidate;
            if (validCardinal(candidate, current)) output.add(candidate);
        }
    }

    /**
     * Diagonals are gated on both adjoining cardinals, so a mob never clips a
     * corner it could not walk around in two moves.
     */
    private void scanCorners(
            SearchNode current, StepContext step, SearchNode[] sides,
            List<SearchNode> output) {
        for (int first = 0; first < sides.length; first++) {
            int second = (first + 1) % sides.length;
            if (!validDiagonalCardinals(
                    current, sides[first], sides[second])) continue;
            Direction a = CARDINAL[first];
            Direction b = CARDINAL[second];
            SearchNode candidate = findAccepted(
                    current.x + a.x() + b.x(), current.y,
                    current.z + a.z() + b.z(), step.height(), step.floor(),
                    a, current.type);
            if (validDiagonal(candidate)
                    && !clearance.swungPanelBlocks(current, candidate)) {
                output.add(candidate);
            }
        }
    }

    /** Overridden where a movement model adds vertical edges. */
    void addVerticalNeighbors(SearchNode current, List<SearchNode> result,
                              int jumpSize, double floor) {
    }

    SearchNode findAccepted(int x, int y, int z, int jumpSize,
                            double sourceFloor, Direction direction,
                            TerrainType sourceType) {
        return edges.resolve(
                x, y, z, jumpSize, sourceFloor, direction, sourceType);
    }

    boolean validCardinal(SearchNode node, SearchNode current) {
        return node != null && !node.settled && !node.hardBlocked
                && (node.malus >= 0 || current.malus < 0)
                && !clearance.swungPanelBlocks(current, node);
    }

    private boolean validDiagonalCardinals(SearchNode current,
                                           SearchNode first, SearchNode second) {
        if (first == null || second == null
                || first.y > current.y || second.y > current.y
                || first.type == TerrainType.WALKABLE_DOOR
                || second.type == TerrainType.WALKABLE_DOOR) return false;
        if (box.width() > 1 && (first.malus > 0 || second.malus > 0)) {
            return false;
        }
        return diagonalSide(current, first, second)
                && diagonalSide(current, second, first);
    }

    /**
     * A side is passable if it is below the current cell, enterable, or one of
     * a pair of fence posts a mob is slim enough to slip between.
     */
    private boolean diagonalSide(SearchNode current, SearchNode side,
                                 SearchNode other) {
        return side.y < current.y || side.malus >= 0
                || (side.type == TerrainType.FENCE
                && other.type == TerrainType.FENCE && box.width() < 0.5);
    }

    private static boolean validDiagonal(SearchNode diagonal) {
        return diagonal != null && !diagonal.settled && !diagonal.hardBlocked
                && diagonal.type != TerrainType.WALKABLE_DOOR
                && diagonal.malus >= 0;
    }

    /**
     * The cell with its cost resolved: baseline terrain cost, then every
     * influence layered on top. A blocking influence latches on the node, so a
     * cell refused once is not re-evaluated by later expansions.
     */
    SearchNode accepted(int x, int y, int z) {
        SearchNode node = node(x, y, z);
        TerrainType type = type(x, y, z);
        node.type = type;
        double malus = profile.malus(type);
        if (malus >= 0 && !influences.isEmpty()) {
            boolean influenceBlocked = false;
            Point projected = new Vec(
                    x + footprintWidth(box) * 0.5, floorLevel(x, y, z),
                    z + footprintDepth(box) * 0.5);
            for (NavigationInfluence influence : influences) {
                InfluenceResult result = influence.evaluate(
                        blocks, projected, box, control);
                if (result.blocked()) {
                    influenceBlocked = true;
                    break;
                }
                malus += result.costDelta();
            }
            malus = influenceBlocked ? -1 : Math.max(0, malus);
            node.hardBlocked |= influenceBlocked;
        }
        node.malus = Math.max(node.malus, malus);
        return node;
    }

    SearchNode node(int x, int y, int z) {
        return nodes.getOrCreate(x, y, z);
    }

    /** Classification is immutable per cell, so it is cached on the node. */
    TerrainType type(int x, int y, int z) {
        SearchNode node = node(x, y, z);
        if (!node.typeComputed) {
            node.classifiedType = classify(x, y, z);
            node.typeComputed = true;
        }
        return node.classifiedType;
    }

    TerrainType classify(int x, int y, int z) {
        return classifier.classifyAnchored(
                blocks, x, y, z, box, profile, control);
    }

    TerrainType classifyAmphibious(int x, int y, int z) {
        return classifier.classifyAnchoredAmphibious(
                blocks, x, y, z, box, profile, control);
    }

    double floorLevel(int x, int y, int z) {
        SearchNode node = node(x, y, z);
        if (node.floorComputed) return node.floor;
        node.floor = computeFloorLevel(x, y, z);
        node.floorComputed = true;
        return node.floor;
    }

    private double computeFloorLevel(int x, int y, int z) {
        if ((profile.canFloat() || isAmphibious())
                && isWater(blocks.getBlock(
                x, y, z, Block.Getter.Condition.TYPE))) return y + 0.5;
        Block support = blocks.getBlock(
                x, y - 1, z, Block.Getter.Condition.TYPE);
        double top = support.collisionShape().relativeEnd().y();
        return y - 1 + (top <= 0 ? 0 : top);
    }

    /**
     * The height a follower is steered to. A climb waypoint sits at the cell
     * itself, and a cell with nothing beneath it keeps its own height rather
     * than projecting onto a floor that is not there.
     */
    double waypointY(int x, int y, int z) {
        if (climb.anchoredAt(x, y, z)) return y;
        Block below = blocks.getBlock(
                x, y - 1, z, Block.Getter.Condition.TYPE);
        return below.air() ? y : floorLevel(x, y, z);
    }

    boolean interrupted() {
        return control.cancelled() || control.timedOut();
    }

    @Override
    PathResult result(PathStatus status, SearchNode end,
                      int visited, int examined) {
        return routes.assemble(status, end, visited, examined);
    }

    static int footprintWidth(BoundingBox box) {
        return Math.max(1, (int) Math.floor(box.width() + 1));
    }

    static int footprintDepth(BoundingBox box) {
        return Math.max(1, (int) Math.floor(box.depth() + 1));
    }

    private static double distance(SearchNode a, SearchNode b) {
        return Math.sqrt((double) (a.x - b.x) * (a.x - b.x)
                + (double) (a.y - b.y) * (a.y - b.y)
                + (double) (a.z - b.z) * (a.z - b.z));
    }

    record Direction(int x, int z) {
    }

    private record StepContext(int height, double floor, boolean onClimbable) {
    }
}
