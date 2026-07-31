package ca.atlasengine.pathfinding.search;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;

import java.util.ArrayList;
import java.util.List;
import java.util.random.RandomGenerator;

/**
 * Captures an ordered, immutable set of nearby graph-node candidates.
 *
 * <p>Sampling belongs on the entity tick thread. Pathfinding workers receive
 * only the resulting integer coordinates and never access mutable entity
 * random state.</p>
 */
public final class StartNodeSampler {
    private StartNodeSampler() {
    }

    public static List<Point> sampleSmallEntity(
            Point position, BoundingBox box, RandomGenerator random, int count) {
        if (position == null || box == null || random == null || count < 0) {
            throw new IllegalArgumentException("invalid sampling arguments");
        }
        double minimumExtent = 1.1F;
        double xPadding = Math.max(0, minimumExtent - box.width());
        double yPadding = Math.max(0, minimumExtent - box.height());
        double zPadding = Math.max(0, minimumExtent - box.depth());
        int minX = floor(position.x() + box.relativeStart().x() - xPadding);
        int minY = floor(position.y() + box.relativeStart().y() - yPadding);
        int minZ = floor(position.z() + box.relativeStart().z() - zPadding);
        int maxX = floor(position.x() + box.relativeEnd().x() + xPadding);
        int maxY = floor(position.y() + box.relativeEnd().y() + yPadding);
        int maxZ = floor(position.z() + box.relativeEnd().z() + zPadding);

        List<Point> candidates = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            candidates.add(new Vec(
                    random.nextInt(minX, maxX + 1),
                    random.nextInt(minY, maxY + 1),
                    random.nextInt(minZ, maxZ + 1)));
        }
        return List.copyOf(candidates);
    }

    private static int floor(double value) {
        return (int) Math.floor(value);
    }
}
