package ca.atlasengine.pathfinding.search;

import net.minestom.server.instance.block.Block;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;

import java.util.List;
import java.util.Objects;
import java.util.Set;
import static ca.atlasengine.pathfinding.internal.Points.finite;

/**
 * Immutable live-entity facts consumed by navigation evaluators.
 *
 * <p>These values must be snapshotted on the entity tick thread before an
 * asynchronous search. Worker threads never inspect or mutate the entity.</p>
 */
public record EntityTraversalState(
        boolean onGround,
        boolean inWater,
        Set<Block> standableFluids,
        int minBuildHeight,
        int maxBuildHeight,
        List<Point> pathfindingStartCandidates
) {
    /** Generous ceiling preventing target-normalization scans from becoming unbounded. */
    public static final int MAX_BUILD_HEIGHT_SPAN = 4096;
    static final int OVERWORLD_MIN_BUILD_HEIGHT = -64;
    static final int OVERWORLD_MAX_BUILD_HEIGHT = 319;

    public static final EntityTraversalState GROUNDED =
            new EntityTraversalState(
                    true, false, Set.of(), OVERWORLD_MIN_BUILD_HEIGHT,
                    OVERWORLD_MAX_BUILD_HEIGHT, List.of());

    public EntityTraversalState(boolean onGround, boolean inWater,
                              Set<Block> standableFluids,
                              int minBuildHeight) {
        this(onGround, inWater, standableFluids, minBuildHeight,
                OVERWORLD_MAX_BUILD_HEIGHT, List.of());
    }

    public EntityTraversalState {
        if (standableFluids == null
                || standableFluids.stream().anyMatch(Objects::isNull)
                || pathfindingStartCandidates == null
                || pathfindingStartCandidates.stream().anyMatch(point ->
                point == null || !finite(point))) {
            throw new IllegalArgumentException("snapshot data");
        }
        standableFluids = Set.copyOf(standableFluids);
        pathfindingStartCandidates = pathfindingStartCandidates.stream()
                .map(point -> (Point) new Vec(
                        point.blockX(), point.blockY(), point.blockZ()))
                .toList();
        long buildHeightSpan = (long) maxBuildHeight - minBuildHeight;
        if (maxBuildHeight < minBuildHeight
                || buildHeightSpan > MAX_BUILD_HEIGHT_SPAN
                || minBuildHeight == Integer.MIN_VALUE
                || maxBuildHeight == Integer.MAX_VALUE) {
            throw new IllegalArgumentException("build height");
        }
    }


    /** Identity comparison, so a fluid's level never changes the answer. */
    public boolean canStandOn(Block block) {
        if (!block.liquid()) return false;
        for (Block fluid : standableFluids) {
            if (block.compare(fluid)) return true;
        }
        return false;
    }
}
