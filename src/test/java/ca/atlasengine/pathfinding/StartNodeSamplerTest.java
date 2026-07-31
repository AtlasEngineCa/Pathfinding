package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.search.StartNodeSampler;import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class StartNodeSamplerTest {
    @Test
    void smallEntitySamplingUsesInflatedInclusiveIntegerBounds() {
        List<Point> candidates = StartNodeSampler.sampleSmallEntity(
                new Vec(-0.1, 12.25, 0.9),
                new BoundingBox(0.6, 0.6, 0.6),
                new Random(918273), 2_000);

        assertEquals(2_000, candidates.size());
        assertTrue(candidates.stream().allMatch(point ->
                point.blockX() >= -1 && point.blockX() <= 0
                        && point.blockY() >= 11 && point.blockY() <= 13
                        && point.blockZ() >= 0 && point.blockZ() <= 1));
        assertTrue(candidates.stream().anyMatch(point -> point.blockX() == -1
                && point.blockY() == 11 && point.blockZ() == 0),
                "floored lower bounds must be inclusive");
        assertTrue(candidates.stream().anyMatch(point -> point.blockX() == 0
                && point.blockY() == 13 && point.blockZ() == 1),
                "floored upper bounds must be inclusive");
    }

    @Test
    void duplicateCandidatesRemainInTheSnapshot() {
        List<Point> candidates = StartNodeSampler.sampleSmallEntity(
                new Vec(0.5, 4, 0.5),
                new BoundingBox(0.6, 0.6, 0.6),
                () -> 0L, 10);

        assertEquals(10, candidates.size());
        assertEquals(1, candidates.stream().distinct().count(),
                "sampling is with replacement and must not deduplicate");
    }
}
