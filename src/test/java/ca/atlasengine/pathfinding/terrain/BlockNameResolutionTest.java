package ca.atlasengine.pathfinding.terrain;

import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Every block named in the terrain data must exist in the linked Minestom.
 *
 * <p>Names are resolved through {@link Block#fromKey(String)} and an unknown
 * one is skipped, which keeps the data forward-compatible with blocks a newer
 * Minecraft adds but also means a typo silently does nothing. This pins the
 * list so a typo fails here instead of quietly dropping a rule. A genuinely
 * future block belongs in the exemptions below, in the open, rather than
 * looking like a resolved name.</p>
 */
class BlockNameResolutionTest {
    private static final List<String> KNOWN_FUTURE_BLOCKS = List.of();

    @Test
    void everyNamedBlockResolvesInTheLinkedMinestom() throws IOException {
        Path source = Path.of("src/main/java/ca/atlasengine/pathfinding/"
                + "terrain/BlockTraversalData.java");
        String text = Files.readString(source);
        Matcher body = Pattern.compile(
                "LAND_NEVER_IDS = Set\\.of\\((.*?)\\);", Pattern.DOTALL)
                .matcher(text);
        assertTrue(body.find(), "the block list moved or was renamed");

        List<String> unresolved = new ArrayList<>();
        Matcher names = Pattern.compile("\"([a-z_]+)\"").matcher(body.group(1));
        int total = 0;
        while (names.find()) {
            String name = names.group(1);
            total++;
            if (Block.fromKey(name) == null
                    && !KNOWN_FUTURE_BLOCKS.contains(name)) {
                unresolved.add(name);
            }
        }
        assertTrue(total > 50, "expected the full list, saw " + total);
        assertEquals(List.of(), unresolved,
                "these names resolve to no block, so their rule never applies");
    }
}
