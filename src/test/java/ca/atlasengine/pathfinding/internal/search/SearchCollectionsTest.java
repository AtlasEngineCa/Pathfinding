package ca.atlasengine.pathfinding.internal.search;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;

class SearchCollectionsTest {
    @Test
    void coordinateTableRetainsFullIntegersAcrossGrowthAndCollisions() {
        CoordinateNodeMap nodes = new CoordinateNodeMap();
        List<SearchNode> inserted = new ArrayList<>();
        for (int index = 0; index < 2_000; index++) {
            int x = index * 65_537;
            int y = Integer.MIN_VALUE + index * 97;
            int z = Integer.MAX_VALUE - index * 131;
            inserted.add(nodes.getOrCreate(x, y, z));
        }
        for (SearchNode node : inserted) {
            assertSame(node, nodes.get(node.x, node.y, node.z));
            assertSame(node, nodes.getOrCreate(node.x, node.y, node.z));
        }
        assertNotSame(nodes.getOrCreate(0, 0, 0),
                nodes.getOrCreate(0, 1 << 16, 0));
    }

    @Test
    void frontierOrdersScoresAndMovesImprovedNodes() {
        SearchFrontier frontier = new SearchFrontier();
        SearchNode first = scored(1, 8);
        SearchNode second = scored(2, 3);
        SearchNode third = scored(3, 12);
        SearchNode fourth = scored(4, 5);
        frontier.add(first);
        frontier.add(second);
        frontier.add(third);
        frontier.add(fourth);
        frontier.lowerScore(third, 1);

        assertEquals(3, frontier.takeBest().x);
        assertEquals(2, frontier.takeBest().x);
        assertEquals(4, frontier.takeBest().x);
        assertEquals(1, frontier.takeBest().x);
    }

    private static SearchNode scored(int x, float score) {
        SearchNode node = new SearchNode(x, 0, 0);
        node.rank = score;
        return node;
    }
}
