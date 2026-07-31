package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.result.PathNode;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.TerrainClassifier;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.collision.CollisionUtils;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;

/**
 * Route-advancement geometry: whether a waypoint counts as reached, whether
 * the follower may cut ahead of it, and which medium it is travelling in.
 */
public final class RouteGeometry {
    private final MovementContext context;
    private final TerrainClassifier terrainClassifier = new TerrainClassifier();

    public RouteGeometry(MovementContext context) {
        this.context = context;
    }

    public boolean isClimbableWaypoint(PathNode node) {
        Entity entity = context.entity();
        return context.profile().groundCapabilities().climbables().enabled()
                && entity.getInstance() != null
                && BlockTraversalData.isClimbableAt(entity.getInstance(),
                node.graphX(), node.graphY(), node.graphZ());
    }

    public boolean reachedWaypoint(PathNode node, Point waypoint,
                                   Point position, boolean climbableWaypoint) {
        Entity entity = context.entity();
        double tolerance = entity.getBoundingBox().width() > 0.75
                ? entity.getBoundingBox().width() / 2
                : 0.75 - entity.getBoundingBox().width() / 2;
        // Path waypoints are shifted for wide entities so their bounding box
        // occupies the evaluator's complete block footprint. The reference
        // follower nevertheless performs its "node reached" check against
        // the graph block's bottom-center, not that shifted move target.
        // Comparing against the move target can pin a wide entity exactly at
        // its collision boundary beside a wall until the timeout expires.
        Point reachedCenter = climbableWaypoint ? waypoint : graphCenter(node);
        double verticalTolerance = node.movement() == PathNode.Movement.CLIMB
                ? 0.02
                : verticalWaypointTolerance(navigationMode());
        boolean reached =
                Math.abs(position.x() - reachedCenter.x()) < tolerance
                && Math.abs(position.z() - reachedCenter.z()) < tolerance
                && Math.abs(position.y() - reachedCenter.y())
                < verticalTolerance;
        if (node.movement() == PathNode.Movement.JUMP) {
            reached &= entity.isOnGround()
                    || context.hasSupportBelow();
        }
        return reached;
    }

    public boolean canSkipAheadOf(PathNode node, PathNode next, Point position) {
        return node.movement() != PathNode.Movement.JUMP
                && node.movement() != PathNode.Movement.CLIMB
                && canCutCorner(node)
                && shouldAdvanceTowardNextNode(position, node, next);
    }

    private NavigationMode navigationMode() {
        return context.profile().mode();
    }

    private static Point graphCenter(PathNode node) {
        return new Vec(
                node.graphX() + 0.5, node.graphY(), node.graphZ() + 0.5);
    }

    public static double verticalWaypointTolerance(NavigationMode mode) {
        return mode == NavigationMode.WATER ? 0.5 : 1.0;
    }

    private boolean canCutCorner(PathNode node) {
        Entity entity = context.entity();
        TerrainType type;
        if (node.movement() == PathNode.Movement.SWIM
                && navigationMode() == NavigationMode.WATER) {
            type = TerrainType.WATER;
        } else if (node.movement() == PathNode.Movement.BREACH) {
            type = TerrainType.BREACH;
        } else {
            Instance instance = entity.getInstance();
            if (instance == null) return false;
            type = terrainClassifier.classifyAnchored(
                    instance, node.graphX(), node.graphY(), node.graphZ(),
                    entity.getBoundingBox(), context.profile().mobProfile());
        }
        return permitsCornerCut(entity.getEntityType(), type);
    }

    public static boolean permitsCornerCut(EntityType entityType, TerrainType type) {
        return type != TerrainType.FIRE_IN_NEIGHBOR
                && type != TerrainType.DAMAGING_IN_NEIGHBOR
                && type != TerrainType.WALKABLE_DOOR
                && (entityType != EntityType.FROG
                || type != TerrainType.WATER_BORDER);
    }

    private boolean shouldAdvanceTowardNextNode(
            Point position, PathNode current, PathNode next) {
        if (next == null) return false;
        Point currentCenter = graphCenter(current);
        if (position.distance(currentCenter) >= 2) return false;
        if (movesThroughVolume()
                && canMoveDirectly(position, current.asVec())
                && canMoveDirectly(position, next.asVec())) {
            return true;
        }

        Point nextCenter = graphCenter(next);
        Vec toCurrent = currentCenter.sub(position).asVec();
        Vec toNext = nextCenter.sub(position).asVec();
        double currentSquared = toCurrent.lengthSquared();
        double nextSquared = toNext.lengthSquared();
        if (nextSquared >= currentSquared && currentSquared >= 0.5) return false;
        if (currentSquared < 1.0e-12 || nextSquared < 1.0e-12) return false;
        return toCurrent.normalize().dot(toNext.normalize()) < 0;
    }

    /**
     * True while the entity travels through a fluid or air volume rather than
     * across ground.
     */
    public boolean movesThroughVolume() {
        return alwaysMovesThroughVolume()
                || navigationMode() == NavigationMode.AMPHIBIOUS
                && isEntityInWater();
    }

    /**
     * Narrower than {@link #movesThroughVolume()}: profiles that navigate a
     * volume regardless of the entity's live water state.
     */
    public boolean alwaysMovesThroughVolume() {
        return navigationMode() == NavigationMode.WATER
                || navigationMode() == NavigationMode.FLYING;
    }

    private boolean canMoveDirectly(Point start, Point destination) {
        Entity entity = context.entity();
        Instance instance = entity.getInstance();
        if (instance == null) return false;
        Point rayStart = start;
        // The reference shortcut test is a collider ray from the temporary
        // navigation position to the destination's mid-body, not a sweep of
        // the entity bounding box. In particular, water navigation supplies
        // a mid-body start. Treating that point as the bottom of a swept box
        // shifts the test upward and can incorrectly skip a clearance-
        // critical ascent node in front of a submerged wall.
        Point rayEnd = destination.add(
                0, entity.getBoundingBox().height() * 0.5, 0);
        Vec displacement = rayEnd.sub(rayStart).asVec();
        var result = CollisionUtils.handlePhysics(
                instance, entity.getChunk(),
                new BoundingBox(0, 0, 0),
                rayStart.asPos(), displacement, null, false);
        if (result.newPosition().distance(rayEnd) > 1.0e-5) return false;
        if (context.profile().directPathIgnoresFluids()) return true;

        // Flying navigation treats fluids as ray blockers.
        int samples = Math.max(1, (int) Math.ceil(displacement.length() * 20));
        for (int i = 0; i <= samples; i++) {
            Point point = rayStart.add(displacement.mul((double) i / samples));
            if (instance.getBlock(point, Block.Getter.Condition.TYPE).liquid()) {
                return false;
            }
        }
        return true;
    }

    public boolean isSpatialNode(PathNode node) {
        return movesThroughVolume()
                || node.movement() == PathNode.Movement.SWIM
                || node.movement() == PathNode.Movement.BREACH
                || node.movement() == PathNode.Movement.FLY;
    }

    public Point waypointTrackingPosition() {
        Entity entity = context.entity();
        Point position = entity.getPosition();
        boolean midBody = navigationMode() == NavigationMode.WATER
                || navigationMode() == NavigationMode.AMPHIBIOUS
                && isEntityInWater();
        return midBody
                ? position.add(0, entity.getBoundingBox().height() * 0.5, 0)
                : position;
    }

    public boolean isEntityInWater() {
        Entity entity = context.entity();
        Instance instance = entity.getInstance();
        if (instance == null) return false;
        return volumeTouchesWater(instance, entity.getPosition(),
                entity.getBoundingBox());
    }

    public static boolean volumeTouchesWater(
            Block.Getter blocks, Point position, BoundingBox box) {
        int minX = (int) Math.floor(position.x() + box.relativeStart().x());
        int maxX = (int) Math.floor(
                position.x() + box.relativeEnd().x() - 1.0e-7);
        int minY = (int) Math.floor(position.y() + box.relativeStart().y());
        int maxY = (int) Math.floor(
                position.y() + box.relativeEnd().y() - 1.0e-7);
        int minZ = (int) Math.floor(position.z() + box.relativeStart().z());
        int maxZ = (int) Math.floor(
                position.z() + box.relativeEnd().z() - 1.0e-7);
        for (int x = minX; x <= maxX; x++) {
            for (int y = minY; y <= maxY; y++) {
                for (int z = minZ; z <= maxZ; z++) {
                    if (BlockTraversalData.hasWaterFluid(blocks.getBlock(
                            x, y, z, Block.Getter.Condition.TYPE))) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
}
