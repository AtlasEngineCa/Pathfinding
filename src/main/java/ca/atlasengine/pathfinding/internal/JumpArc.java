package ca.atlasengine.pathfinding.internal;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.collision.CollisionUtils;
import net.minestom.server.collision.PhysicsResult;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.Chunk;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;
import net.minestom.server.utils.chunk.ChunkCache;

import java.util.function.BooleanSupplier;

/**
 * The single definition of a platform-jump arc, shared by the search that
 * emits jump edges and by the follower that launches them.
 *
 * <p>The two must agree. A refused launch leaves the entity standing on the
 * same block, so the recomputation it triggers re-emits the identical edge
 * and the entity never moves again.</p>
 */
public final class JumpArc {
    /**
     * Vertical drag pulls the real apex earlier than a dragless parabola
     * places it, lifting the flight above the arc by a share of the peak.
     */
    private static final double DRAG_SKEW = 0.42;
    /**
     * The follower spreads the horizontal travel over one tick less than
     * the flight, so the entity is still that tick of descent above the
     * landing when it arrives over it, and a tick of descent grows as the
     * square root of the height fallen.
     */
    private static final double DESCENT_TICK = 0.54;

    private JumpArc() {
    }

    /**
     * A steep arc climbs faster than it advances, so a count taken from the
     * horizontal leaves tall vertical steps between samples. The sweep below
     * covers those steps; point-sampling them did not, and a tall apex
     * stepped the box straight past a block between two samples.
     */
    private static int samples(Point from, Point to) {
        double horizontal = Math.hypot(
                to.x() - from.x(), to.z() - from.z());
        return Math.max(8, (int) Math.ceil(horizontal * 16));
    }

    private static Point sample(Point from, Point to, double apexClearance,
                                int sample, int samples) {
        double t = (double) sample / samples;
        return new Vec(
                from.x() + (to.x() - from.x()) * t,
                from.y() + height(to.y() - from.y(), apexClearance, t),
                from.z() + (to.z() - from.z()) * t);
    }

    /**
     * The height the follower's bisection launches for. Every arc is swept
     * against this, so it is the one definition of how high a jump goes.
     */
    public static double peak(double rise, double apexClearance) {
        return Math.max(0, rise) + Math.max(0.05, apexClearance);
    }

    /**
     * Height above the takeoff at horizontal fraction {@code t}.
     *
     * <p>A chord bowed by {@code apexClearance} tops out at
     * {@code rise/2 + apexClearance}, half a rise below the flight the
     * follower launches, so every upward jump flew through a band nothing
     * had checked. The launch is ballistic and the horizontal speed is
     * constant, which leaves gravity setting only the timescale; measuring
     * the arc in horizontal fraction divides that out. What is left is the
     * parabola through the takeoff and the landing whose peak is
     * {@link #peak}, and it needs no aerodynamics to write down.</p>
     */
    public static double height(double rise, double apexClearance, double t) {
        double peak = peak(rise, apexClearance);
        double apexFraction = 1 / (1 + Math.sqrt(1 - rise / peak));
        double offset = (t - apexFraction) / apexFraction;
        return peak * (1 - offset * offset);
    }

    /**
     * Vertical slack the swept box carries above the arc, so the sweep is
     * an upper envelope of the flight rather than a curve the flight
     * crosses. Sweeping high only costs jumps that would have worked;
     * sweeping low is what puts a mob through a ceiling.
     *
     * <p>Sized for gravity in {@code [0.02, 0.14]} and vertical air
     * resistance in {@code [0.90, 1]}, which contains every entity type
     * Minecraft ships. A mob outside it flies an arc no envelope written
     * from {@code rise} and {@code apexClearance} alone can bound.</p>
     */
    public static double headroom(double rise, double apexClearance) {
        double peak = peak(rise, apexClearance);
        return Math.max(DRAG_SKEW * peak,
                DESCENT_TICK * Math.sqrt(peak - rise));
    }

    /** The entity's own box, grown upward by {@link #headroom}. */
    private static BoundingBox grown(BoundingBox box, Point from, Point to,
                                     double apexClearance) {
        double headroom = headroom(to.y() - from.y(), apexClearance);
        return new BoundingBox(box.relativeStart(), box.relativeEnd()
                .withY(box.relativeEnd().y() + headroom));
    }

    /**
     * Live counterpart reading through a chunk cache, so an arc leaving the
     * loaded area is refused rather than throwing.
     */
    public static boolean clear(Instance instance, Chunk chunk,
                                BoundingBox box, Point from, Point to,
                                double apexClearance) {
        return clear(new ChunkCache(instance, chunk != null
                        ? chunk : instance.getChunkAt(from), Block.STONE),
                box, from, to, apexClearance, null);
    }

    /**
     * Point-sampling the arc lets the box step diagonally past a block corner
     * between two samples, so a swept test is the only one the follower and
     * the planner can both pass.
     *
     * @param budget per-sample search accounting, or null when live
     */
    public static boolean clear(Block.Getter blocks, BoundingBox box,
                                Point from, Point to, double apexClearance,
                                BooleanSupplier budget) {
        BoundingBox grown = grown(box, from, to, apexClearance);
        // A sweep discards whatever the box already overlaps when it starts,
        // and the grown box reaches above the standing one, so the takeoff
        // is the one position no sweep along the arc can judge.
        if (occupied(blocks, grown, from)) return false;
        int samples = samples(from, to);
        Point previous = from;
        for (int sample = 1; sample <= samples; sample++) {
            if (budget != null && !budget.getAsBoolean()) return false;
            Point next = sample(from, to, apexClearance, sample, samples);
            PhysicsResult result = CollisionUtils.handlePhysics(blocks, grown,
                    previous.asPos(), next.sub(previous).asVec(), null, false);
            // A vertical contact on the final segment is the box settling
            // onto the landing surface.
            if (result.newPosition().distance(next) > 1.0e-5
                    || result.collisionX() || result.collisionZ()
                    || result.collisionY() && sample < samples) {
                return false;
            }
            previous = next;
        }
        return true;
    }

    private static boolean occupied(Block.Getter blocks, BoundingBox box,
                                    Point position) {
        var cells = box.getBlocks(position);
        while (cells.hasNext()) {
            var cell = cells.next();
            Vec corner = new Vec(
                    cell.blockX(), cell.blockY(), cell.blockZ());
            if (blocks.getBlock(cell.blockX(), cell.blockY(), cell.blockZ())
                    .collisionShape()
                    .intersectBox(position.sub(corner), box)) {
                return true;
            }
        }
        return false;
    }
}
