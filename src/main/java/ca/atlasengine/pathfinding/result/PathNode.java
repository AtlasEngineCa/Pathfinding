package ca.atlasengine.pathfinding.result;

import net.minestom.server.coordinate.Vec;

import java.util.Objects;

/**
 * An immutable waypoint.
 *
 * <p>{@code x/y/z} are the physical bottom-center coordinates sent to a
 * follower. {@code graphX/Y/Z} retain the evaluator node which produced that
 * waypoint. They differ on slabs, stairs, fluids, airborne starts, and for
 * wide entities whose baseline graph node is a minimum-volume corner.</p>
 */
public final class PathNode {
    public enum Movement {
        WALK,
        STEP_UP,
        JUMP,
        FALL,
        SWIM,
        BREACH,
        FLY,
        CLIMB
    }

    private final double x;
    private final double y;
    private final double z;
    private final Movement movement;
    private final int graphX;
    private final int graphY;
    private final int graphZ;

    /**
     * Compatibility constructor for physical-coordinate pathfinders.
     */
    public PathNode(double x, double y, double z, Movement movement) {
        this(x, y, z, movement,
                (int) Math.floor(x), (int) Math.floor(y), (int) Math.floor(z));
    }

    public PathNode(double x, double y, double z, Movement movement,
                    int graphX, int graphY, int graphZ) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.movement = Objects.requireNonNull(movement, "movement");
        this.graphX = graphX;
        this.graphY = graphY;
        this.graphZ = graphZ;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public double z() {
        return z;
    }

    public Movement movement() {
        return movement;
    }

    public int graphX() {
        return graphX;
    }

    public int graphY() {
        return graphY;
    }

    public int graphZ() {
        return graphZ;
    }

    public Vec asVec() {
        return new Vec(x, y, z);
    }

    public int blockX() {
        return (int) Math.floor(x);
    }

    public int blockY() {
        return (int) Math.floor(y);
    }

    public int blockZ() {
        return (int) Math.floor(z);
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (!(object instanceof PathNode other)) return false;
        return Double.compare(x, other.x) == 0
                && Double.compare(y, other.y) == 0
                && Double.compare(z, other.z) == 0
                && movement == other.movement
                && graphX == other.graphX && graphY == other.graphY
                && graphZ == other.graphZ;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, z, movement, graphX, graphY, graphZ);
    }

    @Override
    public String toString() {
        return "PathNode[x=" + x + ", y=" + y + ", z=" + z
                + ", movement=" + movement + ", graph=("
                + graphX + ',' + graphY + ',' + graphZ + ")]";
    }
}
