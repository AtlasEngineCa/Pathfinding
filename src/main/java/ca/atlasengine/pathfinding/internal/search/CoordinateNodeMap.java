package ca.atlasengine.pathfinding.internal.search;

/** Search-local, allocation-free coordinate table with full integer keys. */
final class CoordinateNodeMap {
    private static final float LOAD = 0.65F;
    private int[] xs = new int[256];
    private int[] ys = new int[256];
    private int[] zs = new int[256];
    private SearchNode[] values = new SearchNode[256];
    private int mask = 255;
    private int resizeAt = (int) (256 * LOAD);
    private int size;

    SearchNode get(int x, int y, int z) {
        int slot = locate(x, y, z);
        return values[slot];
    }

    SearchNode getOrCreate(int x, int y, int z) {
        int slot = locate(x, y, z);
        SearchNode existing = values[slot];
        if (existing != null) return existing;
        SearchNode created = new SearchNode(x, y, z);
        xs[slot] = x;
        ys[slot] = y;
        zs[slot] = z;
        values[slot] = created;
        if (++size >= resizeAt) grow();
        return created;
    }

    private int locate(int x, int y, int z) {
        int slot = mix(x, y, z) & mask;
        while (true) {
            SearchNode value = values[slot];
            if (value == null || xs[slot] == x && ys[slot] == y
                    && zs[slot] == z) return slot;
            slot = slot + 1 & mask;
        }
    }

    private void grow() {
        int[] oldX = xs;
        int[] oldY = ys;
        int[] oldZ = zs;
        SearchNode[] oldValues = values;
        int capacity = oldValues.length << 1;
        xs = new int[capacity];
        ys = new int[capacity];
        zs = new int[capacity];
        values = new SearchNode[capacity];
        mask = capacity - 1;
        resizeAt = (int) (capacity * LOAD);
        for (int index = 0; index < oldValues.length; index++) {
            SearchNode value = oldValues[index];
            if (value == null) continue;
            int slot = locate(oldX[index], oldY[index], oldZ[index]);
            xs[slot] = oldX[index];
            ys[slot] = oldY[index];
            zs[slot] = oldZ[index];
            values[slot] = value;
        }
    }

    private static int mix(int x, int y, int z) {
        int hash = x * 0x8da6b343 ^ y * 0xd8163841 ^ z * 0xcb1ab31f;
        hash ^= hash >>> 16;
        hash *= 0x7feb352d;
        hash ^= hash >>> 15;
        return hash;
    }
}
