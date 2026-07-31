package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.PriorityQueue;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Independent shortest-path audit on a deliberately long, alternating maze.
 * The oracle does not call production classification or neighbor code.
 */
class HardMapOptimalityTest {
    private static final int MIN = 0;
    private static final int MAX = 30;
    private static final int[][] DIRECTIONS = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1},
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    @Test
    void groundSearchMatchesIndependentShortestRouteForMultipleBoundingBoxes() {
        boolean[][] walls = maze();
        TestWorld world = new TestWorld().floor(
                MIN - 2, MAX + 2, MIN - 2, MAX + 2, 0, Block.STONE);
        for (int x = MIN; x <= MAX; x++) {
            for (int z = MIN; z <= MAX; z++) {
                if (walls[x][z]) world.column(x, z, 1, 4, Block.STONE);
            }
        }

        for (double width : new double[]{0.4, 0.8, 1.01, 1.4, 1.8}) {
            BoundingBox box = new BoundingBox(width, 1.8, width);
            int volume = Math.max(1, (int) Math.floor(width + 1));
            Cell start = new Cell(1, 1);
            Cell goal = new Cell(28 - (volume - 1), 28 - (volume - 1));
            double expected = dijkstra(walls, start, goal, volume);
            assertTrue(Double.isFinite(expected), "oracle fixture has no route");

            PathResult result = new DiscreteGroundPathfinder().findPath(world, new Pos(start.x + volume * 0.5, 1,
                            start.z + volume * 0.5), new Pos(goal.x + volume * 0.5, 1,
                            goal.z + volume * 0.5), box, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(160).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);

            assertTrue(result.found(),
                    () -> "width=" + width + ", nodes=" + result.nodes());
            assertLegal(result, walls, volume);
            double actual = graphLength(result, volume);
            System.out.printf(java.util.Locale.ROOT,
                    "HARD-MAP width=%.2f volume=%d expected=%.9f actual=%.9f "
                            + "nodes=%d turns=%d%n",
                    width, volume, expected, actual, result.nodes().size(),
                    turns(result, volume));
            assertEquals(expected, actual, 1.0e-9,
                    () -> "width=" + width + " returned a legal but non-shortest path: "
                            + result.nodes());
        }
    }

    private static boolean[][] maze() {
        boolean[][] wall = new boolean[MAX + 1][MAX + 1];
        for (int i = MIN; i <= MAX; i++) {
            wall[MIN][i] = wall[MAX][i] = true;
            wall[i][MIN] = wall[i][MAX] = true;
        }
        int wallNumber = 0;
        for (int x = 4; x <= 24; x += 4) {
            int gapStart = (wallNumber++ & 1) == 0 ? 23 : 4;
            for (int z = 1; z < MAX; z++) {
                if (z < gapStart || z > gapStart + 3) wall[x][z] = true;
            }
        }
        return wall;
    }

    private static double dijkstra(boolean[][] walls, Cell start,
                                   Cell goal, int volume) {
        double[][] distance = new double[MAX + 1][MAX + 1];
        for (double[] row : distance) Arrays.fill(row, Double.POSITIVE_INFINITY);
        PriorityQueue<Queued> queue = new PriorityQueue<>();
        distance[start.x][start.z] = 0;
        queue.add(new Queued(start, 0));
        while (!queue.isEmpty()) {
            Queued current = queue.poll();
            if (current.distance > distance[current.cell.x][current.cell.z]) continue;
            if (current.cell.equals(goal)) return current.distance;
            for (int[] direction : DIRECTIONS) {
                Cell next = new Cell(current.cell.x + direction[0],
                        current.cell.z + direction[1]);
                if (!open(walls, next, volume)) continue;
                if (direction[0] != 0 && direction[1] != 0
                        && (!open(walls, new Cell(next.x, current.cell.z), volume)
                        || !open(walls, new Cell(current.cell.x, next.z), volume))) {
                    continue;
                }
                double candidate = current.distance
                        + (direction[0] != 0 && direction[1] != 0
                        ? Math.sqrt(2) : 1);
                if (candidate < distance[next.x][next.z]) {
                    distance[next.x][next.z] = candidate;
                    queue.add(new Queued(next, candidate));
                }
            }
        }
        return Double.POSITIVE_INFINITY;
    }

    private static void assertLegal(PathResult result, boolean[][] walls,
                                    int volume) {
        Cell previous = null;
        for (PathNode node : result.nodes()) {
            Cell cell = anchor(node, volume);
            assertTrue(open(walls, cell, volume),
                    () -> "occupied waypoint " + cell);
            if (previous != null) {
                int dx = cell.x - previous.x;
                int dz = cell.z - previous.z;
                assertTrue(Math.abs(dx) <= 1 && Math.abs(dz) <= 1
                        && (dx != 0 || dz != 0), "non-neighbor edge");
                if (dx != 0 && dz != 0) {
                    assertTrue(open(walls, new Cell(cell.x, previous.z), volume));
                    assertTrue(open(walls, new Cell(previous.x, cell.z), volume));
                }
            }
            previous = cell;
        }
    }

    private static double graphLength(PathResult result, int volume) {
        double length = 0;
        for (int i = 1; i < result.nodes().size(); i++) {
            Cell a = anchor(result.nodes().get(i - 1), volume);
            Cell b = anchor(result.nodes().get(i), volume);
            length += Math.hypot(a.x - b.x, a.z - b.z);
        }
        return length;
    }

    private static int turns(PathResult result, int volume) {
        int turns = 0;
        int previousX = 0;
        int previousZ = 0;
        for (int i = 1; i < result.nodes().size(); i++) {
            Cell a = anchor(result.nodes().get(i - 1), volume);
            Cell b = anchor(result.nodes().get(i), volume);
            int dx = b.x - a.x;
            int dz = b.z - a.z;
            if (i > 1 && (dx != previousX || dz != previousZ)) turns++;
            previousX = dx;
            previousZ = dz;
        }
        return turns;
    }

    private static Cell anchor(PathNode node, int volume) {
        return new Cell((int) Math.round(node.x() - volume * 0.5),
                (int) Math.round(node.z() - volume * 0.5));
    }

    private static boolean open(boolean[][] walls, Cell cell, int volume) {
        if (cell.x < MIN || cell.z < MIN
                || cell.x + volume - 1 > MAX
                || cell.z + volume - 1 > MAX) return false;
        for (int x = cell.x; x < cell.x + volume; x++) {
            for (int z = cell.z; z < cell.z + volume; z++) {
                if (walls[x][z]) return false;
            }
        }
        return true;
    }

    private record Cell(int x, int z) {
    }

    private record Queued(Cell cell, double distance)
            implements Comparable<Queued> {
        @Override
        public int compareTo(Queued other) {
            return Double.compare(distance, other.distance);
        }
    }
}
