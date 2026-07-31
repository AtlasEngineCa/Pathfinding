package ca.atlasengine.pathfinding.internal.search;

/**
 * Search-node priority queue backed by a four-way heap. Four children reduce
 * the number of levels traversed by the frequent score-decrease operation,
 * while direct field access keeps the queue free of adapter calls.
 */
final class SearchFrontier {
    private static final int ARITY = 4;
    private SearchNode[] entries = new SearchNode[128];
    private int size;

    boolean isEmpty() {
        return size == 0;
    }

    void add(SearchNode node) {
        if (node.frontierSlot >= 0) {
            throw new IllegalStateException("node already queued");
        }
        if (size == entries.length) {
            SearchNode[] grown = new SearchNode[size << 1];
            System.arraycopy(entries, 0, grown, 0, size);
            entries = grown;
        }
        int insertion = size++;
        entries[insertion] = node;
        node.frontierSlot = insertion;
        moveUp(insertion);
    }

    SearchNode takeBest() {
        SearchNode best = entries[0];
        SearchNode tail = entries[--size];
        entries[size] = null;
        if (size != 0) {
            entries[0] = tail;
            tail.frontierSlot = 0;
            moveDown(0);
        }
        best.frontierSlot = -1;
        return best;
    }

    void lowerScore(SearchNode node, float score) {
        node.rank = score;
        moveUp(node.frontierSlot);
    }

    private void moveUp(int position) {
        SearchNode node = entries[position];
        while (position != 0) {
            int parentPosition = (position - 1) / ARITY;
            SearchNode parent = entries[parentPosition];
            if (node.rank >= parent.rank) break;
            entries[position] = parent;
            parent.frontierSlot = position;
            position = parentPosition;
        }
        entries[position] = node;
        node.frontierSlot = position;
    }

    private void moveDown(int position) {
        SearchNode node = entries[position];
        while (true) {
            int firstChild = position * ARITY + 1;
            if (firstChild >= size) break;
            int bestPosition = firstChild;
            float bestScore = entries[firstChild].rank;
            int limit = Math.min(firstChild + ARITY, size);
            for (int child = firstChild + 1; child < limit; child++) {
                float score = entries[child].rank;
                if (score < bestScore) {
                    bestScore = score;
                    bestPosition = child;
                }
            }
            if (bestScore >= node.rank) break;
            SearchNode child = entries[bestPosition];
            entries[position] = child;
            child.frontierSlot = position;
            position = bestPosition;
        }
        entries[position] = node;
        node.frontierSlot = position;
    }
}
