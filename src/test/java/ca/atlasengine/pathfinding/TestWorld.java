package ca.atlasengine.pathfinding;

import net.minestom.server.instance.block.Block;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.LongAdder;

public final class TestWorld implements Block.Getter, Block.Setter {
    private final Map<Key, Block> blocks = new HashMap<>();
    private final LongAdder reads = new LongAdder();

    public TestWorld floor(int minX, int maxX, int minZ, int maxZ, int y, Block block) {
        for (int x = minX; x <= maxX; x++) {
            for (int z = minZ; z <= maxZ; z++) set(x, y, z, block);
        }
        return this;
    }

    public TestWorld set(int x, int y, int z, Block block) {
        Key key = new Key(x, y, z);
        if (block == Block.AIR) blocks.remove(key);
        else blocks.put(key, block);
        return this;
    }

    public TestWorld column(int x, int z, int minY, int maxY, Block block) {
        for (int y = minY; y <= maxY; y++) set(x, y, z, block);
        return this;
    }

    @Override
    public void setBlock(int x, int y, int z, Block block) {
        set(x, y, z, block);
    }

    @Override
    public Block getBlock(int x, int y, int z, Condition condition) {
        reads.increment();
        return blocks.getOrDefault(new Key(x, y, z), Block.AIR);
    }

    public long reads() {
        return reads.sum();
    }

    private record Key(int x, int y, int z) {
    }
}
