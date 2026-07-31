package ca.atlasengine.pathfinding.terrain;

import java.util.Collections;
import java.util.EnumSet;
import java.util.Set;

/**
 * Which openable block families a mob may toggle, and whether it closes them
 * again behind itself.
 *
 * <p>Built-in mobs only ever open doors. {@code InteractWithDoor} and
 * {@code OpenDoorGoal} act on {@code DoorBlock} states inside
 * {@code BlockTags.MOB_INTERACTABLE_DOORS}, and no goal or behavior toggles a
 * trapdoor or a fence gate; ordinary ground navigation prices a closed fence
 * gate as an ordinary impassable {@code FENCE}. {@link #STANDARD} therefore
 * covers doors alone and never closes, which is exactly what
 * {@code canOpenDoors} has always meant here. Trapdoors, fence gates, and
 * closing behind are library extensions that stay off until an integration
 * asks for them.</p>
 */
public record BlockManipulationCapabilities(
        Set<OpenableBlockFamily> families,
        boolean closesBehind,
        double closeRange
) {
    /** {@code InteractWithDoor.SKIP_CLOSING_DOOR_IF_FURTHER_AWAY_THAN}. */
    public static final double DEFAULT_CLOSE_RANGE = 3.0;

    public static final BlockManipulationCapabilities DISABLED =
            new BlockManipulationCapabilities(
                    Set.of(), false, DEFAULT_CLOSE_RANGE);
    public static final BlockManipulationCapabilities STANDARD =
            new BlockManipulationCapabilities(
                    Set.of(OpenableBlockFamily.DOOR), false,
                    DEFAULT_CLOSE_RANGE);

    public BlockManipulationCapabilities {
        if (families == null) throw new IllegalArgumentException("families");
        if (!Double.isFinite(closeRange) || closeRange <= 0
                || closeRange > 64) {
            throw new IllegalArgumentException("closeRange");
        }
        families = families.isEmpty()
                ? Set.of()
                : Collections.unmodifiableSet(EnumSet.copyOf(families));
    }

    public static BlockManipulationCapabilities of(
            OpenableBlockFamily... families) {
        return new BlockManipulationCapabilities(
                Set.of(families), false, DEFAULT_CLOSE_RANGE);
    }

    public boolean enabled() {
        return !families.isEmpty();
    }

    public boolean manipulates(OpenableBlockFamily family) {
        return family != null && families.contains(family);
    }

    public BlockManipulationCapabilities withFamily(
            OpenableBlockFamily family, boolean allowed) {
        if (family == null) throw new IllegalArgumentException("family");
        if (manipulates(family) == allowed) return this;
        EnumSet<OpenableBlockFamily> updated =
                EnumSet.noneOf(OpenableBlockFamily.class);
        updated.addAll(families);
        if (allowed) updated.add(family);
        else updated.remove(family);
        return new BlockManipulationCapabilities(
                updated, closesBehind, closeRange);
    }

    /** Closes manipulated blocks again at the baseline three-block range. */
    public BlockManipulationCapabilities closingBehind() {
        return closingBehind(DEFAULT_CLOSE_RANGE);
    }

    public BlockManipulationCapabilities closingBehind(double range) {
        return new BlockManipulationCapabilities(families, true, range);
    }
}
