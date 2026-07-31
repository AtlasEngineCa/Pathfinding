package ca.atlasengine.pathfinding.terrain;

import ca.atlasengine.pathfinding.search.SearchControl;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.block.Block;

import java.util.EnumSet;
import java.util.Set;

/**
 * Stateless Minestom block-to-navigation-terrain classifier. Shared entry
 * points are static so movement evaluators can reuse them without retaining
 * classifier state.
 */
public final class TerrainClassifier {
    private static final int[][] AXIAL_DIRECTIONS = {
            {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
            {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };
    public TerrainType classify(Block.Getter blocks, Point point,
                                    BoundingBox box, MobTraversalProfile profile) {
        int minX = (int) Math.floor(point.x() - box.width() / 2);
        int minY = point.blockY();
        int minZ = (int) Math.floor(point.z() - box.depth() / 2);
        return classifyAnchored(blocks, minX, minY, minZ, box, profile);
    }

    /**
     * Classifies an integer minimum-corner graph node. The graph coordinate is
     * not the entity's physical center.
     */
    public TerrainType classifyAnchored(Block.Getter blocks,
                                            int minX, int minY, int minZ,
                                            BoundingBox box,
                                            MobTraversalProfile profile) {
        return classifyAnchored(blocks, minX, minY, minZ, box, profile,
                false, SearchControl.NONE);
    }

    public TerrainType classifyAnchored(
            Block.Getter blocks, int minX, int minY, int minZ,
            BoundingBox box, MobTraversalProfile profile,
            SearchControl control) {
        return classifyAnchored(
                blocks, minX, minY, minZ, box, profile, false, control);
    }

    public TerrainType classifyAnchoredAmphibious(
            Block.Getter blocks, int minX, int minY, int minZ,
            BoundingBox box, MobTraversalProfile profile) {
        return classifyAnchored(blocks, minX, minY, minZ, box, profile,
                true, SearchControl.NONE);
    }

    public TerrainType classifyAnchoredAmphibious(
            Block.Getter blocks, int minX, int minY, int minZ,
            BoundingBox box, MobTraversalProfile profile,
            SearchControl control) {
        return classifyAnchored(
                blocks, minX, minY, minZ, box, profile, true, control);
    }

    private TerrainType classifyAnchored(
            Block.Getter blocks, int minX, int minY, int minZ,
            BoundingBox box, MobTraversalProfile profile,
            boolean waterBorderAtCellLevel, SearchControl control) {
        TerrainClassification classification = profile.classification();
        Set<TerrainType> types = EnumSet.noneOf(TerrainType.class);
        int sizeX = Math.max(1, (int) Math.floor(box.width() + 1));
        int sizeY = Math.max(1, (int) Math.floor(box.height() + 1));
        int sizeZ = Math.max(1, (int) Math.floor(box.depth() + 1));
        // Where the entity stands once it occupies this node, which is what
        // an openable's swung panel has to leave room for. Only a profile
        // that swings one ever needs to know.
        Point rest = profile.canPassDoors()
                && profile.blockManipulation().enabled()
                ? new Vec(minX + sizeX * 0.5, minY, minZ + sizeZ * 0.5)
                : null;
        Point stand = new Vec(minX + sizeX * 0.5,
                floorLevel(blocks, minX, minY, minZ, profile,
                        waterBorderAtCellLevel),
                minZ + sizeZ * 0.5);
        TerrainType current = TerrainType.BLOCKED;
        int scanned = 0;
        for (int x = minX; x < minX + sizeX; x++) {
            for (int y = minY; y < minY + sizeY; y++) {
                for (int z = minZ; z < minZ + sizeZ; z++) {
                    if (control.interruptible() && (scanned++ & 63) == 0
                            && (control.cancelled() || control.timedOut())) {
                        return TerrainType.BLOCKED;
                    }
                    Block block = blocks.getBlock(
                            x, y, z, Block.Getter.Condition.TYPE);
                    TerrainType cell = transform(cellType(blocks, block,
                            x, y, z, profile, classification,
                            waterBorderAtCellLevel, rest, box), profile);
                    if (x == minX && y == minY && z == minZ) current = cell;
                    types.add(profile.malus(cell) >= 0 && obstructs(
                            blocks, block, x, y, z, minY, stand, box, profile)
                            ? TerrainType.BLOCKED : cell);
                }
            }
        }
        return select(types, current, profile, box);
    }

    /**
     * The height the entity's box rests at once it occupies this node,
     * matching the ground search's own floor level.
     */
    private static double floorLevel(
            Block.Getter blocks, int x, int y, int z,
            MobTraversalProfile profile, boolean amphibious) {
        if ((profile.canFloat() || amphibious)
                && BlockTraversalData.hasWaterFluid(blocks.getBlock(
                        x, y, z, Block.Getter.Condition.TYPE))) {
            return y + 0.5;
        }
        double top = blocks.getBlock(x, y - 1, z, Block.Getter.Condition.TYPE)
                .collisionShape().relativeEnd().y();
        return y - 1 + (top <= 0 ? 0 : top);
    }

    /**
     * Whether a cell's shape stands inside the entity instead of under it.
     * {@link BlockTraversalData#isLandPathfindable} answers whether feet may
     * occupy a cell. It is evaluated for every cell the box spans, including
     * head room; a shape rising more than one step above the floor the entity
     * stands on is met by shins, chest, or head instead, so it is measured
     * against the box itself.
     *
     * <p>A node type carries no direction, so a shape that walls off one
     * horizontal axis refuses the cell only where the entity could have
     * crossed on that axis at all. A cell walled on both sides of an axis was
     * never crossable that way, which is what keeps an open door's panel out
     * of its own doorway.</p>
     */
    private static boolean obstructs(
            Block.Getter blocks, Block block, int x, int y, int z, int footY,
            Point stand, BoundingBox box, MobTraversalProfile profile) {
        if (block.air()) return false;
        // A climbable column is held, not walked past, and the climb waypoint
        // stands the box outside the climb face rather than in the cell.
        if (BlockTraversalData.isClimbable(block)
                && BlockTraversalData.isClimbableAt(blocks, x, footY, z)) {
            return false;
        }
        // An openable this profile swings is already judged on its panel, by
        // the resting-box test here and the swept one on every edge, so it
        // keeps that answer rather than being judged twice.
        if (profile.canPassDoors() && profile.blockManipulation().manipulates(
                BlockTraversalData.openableFamily(block))) return false;
        var shape = block.collisionShape();
        if (y + shape.relativeEnd().y() <= stand.y() + 1) return false;
        Point relative = stand.sub(x, y, z);
        if (shape.intersectBox(relative, box)) return true;
        if (crossable(blocks, x, y, z, 1, 0) && shape.intersectBox(
                new Vec(0.5, stand.y() - y, stand.z() - z),
                new BoundingBox(1, box.height(), box.depth()))) {
            return true;
        }
        return crossable(blocks, x, y, z, 0, 1) && shape.intersectBox(
                new Vec(stand.x() - x, stand.y() - y, 0.5),
                new BoundingBox(box.width(), box.height(), 1));
    }

    /** Whether both cells beside this one along an axis are enterable. */
    private static boolean crossable(Block.Getter blocks, int x, int y, int z,
                                     int stepX, int stepZ) {
        return BlockTraversalData.isLandPathfindable(blocks.getBlock(
                x + stepX, y, z + stepZ, Block.Getter.Condition.TYPE))
                && BlockTraversalData.isLandPathfindable(blocks.getBlock(
                x - stepX, y, z - stepZ, Block.Getter.Condition.TYPE));
    }

    private TerrainType cellType(
            Block.Getter blocks, Block block, int x, int y, int z,
            MobTraversalProfile profile, TerrainClassification classification,
            boolean waterBorderAtCellLevel, Point rest, BoundingBox box) {
        TerrainType type = pathTypeAt(
                blocks, block, x, y, z, profile, classification, rest, box);
        if (!waterBorderAtCellLevel || type != TerrainType.WATER) return type;
        for (int[] direction : AXIAL_DIRECTIONS) {
            if (raw(blocks.getBlock(
                    x + direction[0], y + direction[1], z + direction[2],
                    Block.Getter.Condition.TYPE),
                    classification) == TerrainType.BLOCKED) {
                return TerrainType.WATER_BORDER;
            }
        }
        return type;
    }

    /**
     * The baseline classification of one block without consulting a
     * {@link TerrainClassification}. Search requests use the overload below.
     */
    public static TerrainType raw(Block block) {
        if (block.air()) return TerrainType.OPEN;
        if (BlockTraversalData.isTrapdoor(block)
                || block.compare(Block.LILY_PAD)
                || block.compare(Block.BIG_DRIPLEAF)) {
            return TerrainType.TRAPDOOR;
        }
        if (block.compare(Block.POWDER_SNOW)) return TerrainType.POWDER_SNOW;
        if (block.compare(Block.CACTUS)
                || block.compare(Block.SWEET_BERRY_BUSH)) {
            return TerrainType.DAMAGING;
        }
        if (block.compare(Block.HONEY_BLOCK)) return TerrainType.STICKY_HONEY;
        if (block.compare(Block.COCOA)) return TerrainType.COCOA;
        // BlockTags.SPELEOTHEMS in the bundled 26.2 data contains both entries.
        if (block.compare(Block.WITHER_ROSE)
                || BlockTraversalData.isSpeleothem(block)) {
            return TerrainType.DAMAGE_CAUTIOUS;
        }
        if (block.compare(Block.LAVA)) return TerrainType.LAVA;
        if (BlockTraversalData.isBurning(block)) return TerrainType.FIRE;
        if (BlockTraversalData.isDoor(block)) {
            if (Boolean.parseBoolean(property(block, "open", "false"))) {
                return TerrainType.DOOR_OPEN;
            }
            return BlockTraversalData.isHandOpenableDoor(block)
                    ? TerrainType.DOOR_WOOD_CLOSED
                    : TerrainType.DOOR_IRON_CLOSED;
        }
        if (BlockTraversalData.isRail(block)) return TerrainType.RAIL;
        if (BlockTraversalData.isLeaves(block)) return TerrainType.LEAVES;
        if (BlockTraversalData.isFence(block)
                || (BlockTraversalData.isFenceGate(block)
                    && !Boolean.parseBoolean(property(block, "open", "false")))) {
            return TerrainType.FENCE;
        }
        if (block.compare(Block.WATER) || block.liquid()) {
            return TerrainType.WATER;
        }
        return BlockTraversalData.isLandPathfindable(block)
                ? TerrainType.OPEN : TerrainType.BLOCKED;
    }

    /**
     * The classification a search reads: the hook first, the baseline mapping
     * wherever it declines. A profile without a hook never reaches the call.
     */
    public static TerrainType raw(
            Block block, TerrainClassification classification) {
        if (classification != TerrainClassification.NONE) {
            TerrainType named = classification.classify(block);
            if (named != null) return named;
        }
        return raw(block);
    }

    public static TerrainType transform(TerrainType type, MobTraversalProfile profile) {
        if (type == TerrainType.DOOR_WOOD_CLOSED
                && profile.canOpenDoors() && profile.canPassDoors()) {
            return TerrainType.WALKABLE_DOOR;
        }
        if (type == TerrainType.DOOR_OPEN && !profile.canPassDoors()) {
            return TerrainType.BLOCKED;
        }
        if (type == TerrainType.RAIL && !profile.currentlyOnRail()) {
            return TerrainType.UNPASSABLE_RAIL;
        }
        return type;
    }

    private static String property(
            Block block, String name, String defaultValue) {
        String value = block.getProperty(name);
        return value == null ? defaultValue : value;
    }

    private TerrainType pathTypeAt(Block.Getter blocks, Block block,
                                   int x, int y, int z,
                                   MobTraversalProfile profile,
                                   TerrainClassification classification,
                                   Point rest, BoundingBox box) {
        TerrainType type = raw(block, classification);
        // An openable this profile toggles is read in the state the follower
        // leaves it in. The swung panel is a shape, not air, so a cell whose
        // panel stands inside the entity box is no cell to route through,
        // whichever path type the closed block carries.
        if (rest != null && BlockTraversalData.obstructsWhenOpen(
                block, profile.blockManipulation(),
                rest.sub(x, y, z), box)) {
            return TerrainType.BLOCKED;
        }
        // Closed doors reach WALKABLE_DOOR through transform(); the opt-in
        // families have no dedicated terrain type, so they join it
        // here, where the block state is still available. The support block
        // below is deliberately left at its ordinary terrain type.
        if ((type == TerrainType.TRAPDOOR || type == TerrainType.FENCE)
                && profile.canPassDoors()
                && profile.blockManipulation().manipulates(
                        BlockTraversalData.openableFamily(block))) {
            return TerrainType.WALKABLE_DOOR;
        }
        if (type != TerrainType.OPEN) return type;
        TerrainType support = raw(blocks.getBlock(
                x, y - 1, z, Block.Getter.Condition.TYPE), classification);
        return standingOn(support, blocks, x, y, z, classification);
    }

    private TerrainType standingOn(TerrainType support,
                                       Block.Getter blocks, int x, int y, int z,
                                       TerrainClassification classification) {
        return switch (support) {
            case OPEN, WATER, LAVA, WALKABLE -> TerrainType.OPEN;
            case FIRE -> TerrainType.FIRE;
            case DAMAGING -> TerrainType.DAMAGING;
            case STICKY_HONEY -> TerrainType.STICKY_HONEY;
            case POWDER_SNOW -> TerrainType.ON_TOP_OF_POWDER_SNOW;
            case DAMAGE_CAUTIOUS -> TerrainType.DAMAGE_CAUTIOUS;
            case TRAPDOOR -> TerrainType.ON_TOP_OF_TRAPDOOR;
            default -> neighborType(blocks, x, y, z, classification);
        };
    }

    static TerrainType neighborType(Block.Getter blocks, int x, int y, int z,
                                    TerrainClassification classification) {
        return neighborType(
                blocks, x, y, z, TerrainType.WALKABLE, classification);
    }

    public static TerrainType neighborType(Block.Getter blocks, int x, int y, int z,
                                           TerrainType fallback,
                                           TerrainClassification classification) {
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    if (dx == 0 && dz == 0) continue;
                    TerrainType type = raw(blocks.getBlock(
                            x + dx, y + dy, z + dz,
                            Block.Getter.Condition.TYPE), classification);
                    if (type == TerrainType.DAMAGING) {
                        return TerrainType.DAMAGING_IN_NEIGHBOR;
                    }
                    if (type == TerrainType.FIRE || type == TerrainType.LAVA) {
                        return TerrainType.FIRE_IN_NEIGHBOR;
                    }
                    if (type == TerrainType.WATER) {
                        return TerrainType.WATER_BORDER;
                    }
                    if (type == TerrainType.DAMAGE_CAUTIOUS) {
                        return TerrainType.DAMAGE_CAUTIOUS;
                    }
                }
            }
        }
        return fallback;
    }

    public static TerrainType select(Set<TerrainType> types,
                                          TerrainType current,
                                          MobTraversalProfile profile, BoundingBox box) {
        if (types.contains(TerrainType.FENCE)) return TerrainType.FENCE;
        if (types.contains(TerrainType.UNPASSABLE_RAIL)) {
            return TerrainType.UNPASSABLE_RAIL;
        }
        TerrainType selected = TerrainType.BLOCKED;
        double highest = profile.malus(selected);
        for (TerrainType type : types) {
            double malus = profile.malus(type);
            if (malus < 0) return type;
            if (malus >= highest) {
                selected = type;
                highest = malus;
            }
        }
        if ((int) Math.floor(box.width() + 1) > 1) {
            boolean currentIsCheaper = profile.malus(current) < highest;
            if (currentIsCheaper
                    && profile.malus(TerrainType.BIG_MOBS_CLOSE_TO_DANGER) < highest) {
                return TerrainType.BIG_MOBS_CLOSE_TO_DANGER;
            }
        } else if (current == TerrainType.OPEN
                && selected != TerrainType.OPEN && highest == 0) {
            return TerrainType.OPEN;
        }
        return selected;
    }

}
