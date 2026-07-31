package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.ReachedCells;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;

import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Set;

/**
 * Opens the openable blocks a follower walks into, for profiles allowed to
 * use them, and closes them again for profiles that ask for it.
 *
 * <p>The closing half follows {@code InteractWithDoor}: a remembered block is
 * held open while it still stands under the current or previously consumed
 * waypoint's footprint, is forgotten unchanged once the mob is further away
 * than the configured range, and is closed otherwise. Nothing is remembered
 * at all unless the profile closes behind itself, so a default profile keeps
 * opening doors exactly as before.</p>
 *
 * <p>The world is any readable and writable block source rather than an
 * {@code Instance}, so a test can hold this and the planner to the same
 * cells instead of restating what this does.</p>
 */
public final class BlockManipulator {
    private final Set<Vec> opened = new LinkedHashSet<>();
    private ReachedCells current;
    private ReachedCells previous;

    public <W extends Block.Getter & Block.Setter> void openAt(
            W world, MobTraversalProfile profile,
            Point waypoint, BoundingBox box) {
        BlockManipulationCapabilities manipulation =
                profile.blockManipulation();
        if (world == null || !manipulation.enabled()
                || !profile.canPassDoors()) return;
        ReachedCells reached = ReachedCells.at(waypoint, box);
        if (!reached.equals(current)) {
            previous = current;
            current = reached;
        }
        open(world, profile, waypoint, box,
                manipulation.closesBehind() ? opened : null);
    }

    /**
     * Opens one waypoint's cells without remembering them. A platform jump
     * calls this on its landing before it sweeps the arc, so the search may
     * read those cells as open; the controller has already run {@link
     * #openAt} for the same waypoint, which is where a closing profile
     * remembers them.
     */
    public static <W extends Block.Getter & Block.Setter> void openLanding(
            W world, MobTraversalProfile profile,
            Point waypoint, BoundingBox box) {
        open(world, profile, waypoint, box, null);
    }

    private static <W extends Block.Getter & Block.Setter> void open(
            W world, MobTraversalProfile profile,
            Point waypoint, BoundingBox box, Set<Vec> remembered) {
        BlockManipulationCapabilities manipulation =
                profile.blockManipulation();
        if (world == null || !manipulation.enabled()
                || !profile.canPassDoors()) return;
        ReachedCells cells = ReachedCells.at(waypoint, box);
        for (int x = cells.minX(); x <= cells.maxX(); x++) {
            for (int z = cells.minZ(); z <= cells.maxZ(); z++) {
                for (int y = cells.minY(); y <= cells.maxY(); y++) {
                    Block block = world.getBlock(x, y, z);
                    if (!BlockTraversalData.isClosedManipulable(
                            block, manipulation)) {
                        continue;
                    }
                    world.setBlock(
                            x, y, z, block.withProperty("open", "true"));
                    if (remembered != null) remembered.add(new Vec(x, y, z));
                }
            }
        }
    }

    public <W extends Block.Getter & Block.Setter> void closeBehind(
            W world, MobTraversalProfile profile, Point position) {
        if (opened.isEmpty() || world == null) return;
        BlockManipulationCapabilities manipulation =
                profile.blockManipulation();
        if (!manipulation.closesBehind()) {
            opened.clear();
            return;
        }
        Iterator<Vec> iterator = opened.iterator();
        while (iterator.hasNext()) {
            Vec block = iterator.next();
            if (heldOpen(block)) continue;
            iterator.remove();
            if (position.distance(block.add(0.5, 0.5, 0.5))
                    > manipulation.closeRange()) continue;
            Block state = world.getBlock(
                    block.blockX(), block.blockY(), block.blockZ());
            if (BlockTraversalData.openableFamily(state) == null
                    || !BlockTraversalData.isOpen(state)) continue;
            world.setBlock(block.blockX(), block.blockY(), block.blockZ(),
                    state.withProperty("open", "false"));
        }
    }

    private boolean heldOpen(Vec block) {
        return underFoot(block, current) || underFoot(block, previous);
    }

    private static boolean underFoot(Vec block, ReachedCells reached) {
        return reached != null
                && block.blockX() >= reached.minX()
                && block.blockX() <= reached.maxX()
                && block.blockZ() >= reached.minZ()
                && block.blockZ() <= reached.maxZ();
    }
}
