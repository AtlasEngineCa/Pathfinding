package ca.atlasengine.pathfinding.terrain;

import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.instance.block.Block;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.Objects;

/**
 * Immutable movement capabilities and per-terrain cost overrides.
 */
public final class MobTraversalProfile {
    public static final MobTraversalProfile DEFAULT = builder("default").build();
    public static final MobTraversalProfile FLOATING_DEFAULT = builder("floating_default")
            .from(DEFAULT)
            .canFloat(true)
            .build();
    public static final MobTraversalProfile ANIMAL = builder("animal")
            .malus(TerrainType.FIRE_IN_NEIGHBOR, 16)
            .malus(TerrainType.FIRE, -1)
            .build();
    /**
     * Animal's constructor maluses plus the evaluator flag installed by
     * {@code FloatGoal}. Not every Animal registers that goal.
     */
    public static final MobTraversalProfile FLOATING_ANIMAL = builder("floating_animal")
            .from(ANIMAL)
            .canFloat(true)
            .build();
    public static final MobTraversalProfile VILLAGER = builder("villager")
            .from(ANIMAL)
            .canOpenDoors(true)
            .canFloat(true)
            .build();
    public static final MobTraversalProfile STRIDER = builder("strider")
            .malus(TerrainType.WATER, -1)
            .malus(TerrainType.LAVA, 0)
            .malus(TerrainType.FIRE_IN_NEIGHBOR, 0)
            .malus(TerrainType.FIRE, 0)
            .standsOnLava(true)
            .build();
    public static final MobTraversalProfile BLAZE = builder("blaze")
            .malus(TerrainType.WATER, -1)
            .malus(TerrainType.LAVA, 8)
            .malus(TerrainType.FIRE_IN_NEIGHBOR, 0)
            .malus(TerrainType.FIRE, 0)
            .build();
    public static final MobTraversalProfile ENDERMAN = builder("enderman")
            .malus(TerrainType.WATER, -1)
            .canFloat(true)
            .build();
    public static final MobTraversalProfile WATER_ANIMAL = builder("water_animal")
            .malus(TerrainType.WATER, 0)
            .canPassDoors(false)
            .build();
    public static final MobTraversalProfile NAUTILUS = builder("nautilus")
            .from(ANIMAL)
            .malus(TerrainType.WATER, 0)
            .canPassDoors(false)
            .build();
    public static final MobTraversalProfile AMPHIBIOUS = builder("amphibious")
            .malus(TerrainType.WATER, 0)
            .build();
    public static final MobTraversalProfile AXOLOTL = builder("axolotl")
            .from(ANIMAL)
            .malus(TerrainType.WATER, 0)
            .build();
    public static final MobTraversalProfile TURTLE = builder("turtle")
            .from(ANIMAL)
            .malus(TerrainType.WATER, 0)
            .malus(TerrainType.DOOR_IRON_CLOSED, -1)
            .malus(TerrainType.DOOR_WOOD_CLOSED, -1)
            .malus(TerrainType.DOOR_OPEN, -1)
            .build();
    public static final MobTraversalProfile FROG = builder("frog")
            .from(ANIMAL)
            .malus(TerrainType.WATER, 4)
            .malus(TerrainType.TRAPDOOR, -1)
            .prefersJumpToPreferredBlocks(true)
            .build();
    public static final MobTraversalProfile FOX = builder("fox")
            .from(ANIMAL)
            .malus(TerrainType.DAMAGING_IN_NEIGHBOR, 0)
            .malus(TerrainType.DAMAGING, 0)
            .canFloat(true)
            .build();
    public static final MobTraversalProfile WARDEN = builder("warden")
            .malus(TerrainType.UNPASSABLE_RAIL, 0)
            .malus(TerrainType.DAMAGING, 8)
            .malus(TerrainType.POWDER_SNOW, 8)
            .malus(TerrainType.LAVA, 8)
            .malus(TerrainType.FIRE, 0)
            .malus(TerrainType.FIRE_IN_NEIGHBOR, 0)
            .canFloat(true)
            .build();
    public static final MobTraversalProfile BEE = builder("bee")
            .from(ANIMAL)
            .malus(TerrainType.FIRE, -1)
            .malus(TerrainType.WATER, -1)
            .malus(TerrainType.WATER_BORDER, 16)
            .malus(TerrainType.COCOA, -1)
            .malus(TerrainType.FENCE, -1)
            .canFloat(true)
            .build();
    public static final MobTraversalProfile PARROT = builder("parrot")
            .from(ANIMAL)
            .malus(TerrainType.FIRE_IN_NEIGHBOR, -1)
            .malus(TerrainType.FIRE, -1)
            .malus(TerrainType.COCOA, -1)
            .canFloat(true)
            .build();
    public static final MobTraversalProfile SNIFFER = builder("sniffer")
            .from(ANIMAL)
            .malus(TerrainType.WATER, -1)
            .malus(TerrainType.ON_TOP_OF_POWDER_SNOW, -1)
            .malus(TerrainType.DAMAGE_CAUTIOUS, -1)
            .canFloat(true)
            .build();
    public static final MobTraversalProfile GOAT = builder("goat")
            .from(ANIMAL)
            .malus(TerrainType.POWDER_SNOW, -1)
            .malus(TerrainType.ON_TOP_OF_POWDER_SNOW, -1)
            .canFloat(true)
            .build();
    public static final MobTraversalProfile WOLF = builder("wolf")
            .from(ANIMAL)
            .malus(TerrainType.POWDER_SNOW, -1)
            .malus(TerrainType.ON_TOP_OF_POWDER_SNOW, -1)
            .canFloat(true)
            .build();
    public static final MobTraversalProfile CAMEL = builder("camel")
            .from(ANIMAL)
            .canFloat(true)
            .canWalkOverFences(true)
            .build();
    public static final MobTraversalProfile CHICKEN = builder("chicken")
            .from(ANIMAL)
            .malus(TerrainType.WATER, 0)
            .canFloat(true)
            .build();
    public static final MobTraversalProfile RAVAGER = builder("ravager")
            .malus(TerrainType.LEAVES, 0)
            .canFloat(true)
            .build();
    public static final MobTraversalProfile WITHER_SKELETON = builder("wither_skeleton")
            .malus(TerrainType.LAVA, 8)
            .build();
    public static final MobTraversalProfile ZOMBIFIED_PIGLIN = builder("zombified_piglin")
            .malus(TerrainType.LAVA, 8)
            .build();
    public static final MobTraversalProfile PIGLIN = builder("piglin")
            .malus(TerrainType.FIRE_IN_NEIGHBOR, 16)
            .malus(TerrainType.FIRE, -1)
            .canOpenDoors(true)
            .build();
    public static final MobTraversalProfile ALLAY = builder("allay")
            .canFloat(true)
            .build();
    public static final MobTraversalProfile WITHER = builder("wither")
            .canFloat(true)
            .build();
    public static final MobTraversalProfile VEX = builder("vex")
            .canFloat(true)
            .build();
    public static final MobTraversalProfile HAPPY_GHAST = builder("happy_ghast")
            .from(ANIMAL)
            .canFloat(true)
            .build();
    public static final MobTraversalProfile BREEZE = builder("breeze")
            .malus(TerrainType.ON_TOP_OF_TRAPDOOR, -1)
            .malus(TerrainType.FIRE, -1)
            .build();
    public static final MobTraversalProfile COPPER_GOLEM = builder("copper_golem")
            .malus(TerrainType.FIRE_IN_NEIGHBOR, 16)
            .malus(TerrainType.DAMAGING_IN_NEIGHBOR, 16)
            .malus(TerrainType.FIRE, -1)
            .canOpenDoors(true)
            .build();
    public static final MobTraversalProfile CREAKING = builder("creaking")
            .malus(TerrainType.DAMAGING, 8)
            .malus(TerrainType.POWDER_SNOW, 8)
            .malus(TerrainType.LAVA, 8)
            .malus(TerrainType.FIRE, 0)
            .malus(TerrainType.FIRE_IN_NEIGHBOR, 0)
            .canFloat(true)
            .build();

    private final String name;
    private final Map<TerrainType, Double> maluses;
    private final boolean canPassDoors;
    private final BlockManipulationCapabilities blockManipulation;
    private final boolean canFloat;
    private final boolean canWalkOverFences;
    private final boolean currentlyOnRail;
    private final boolean prefersJumpToPreferredBlocks;
    private final boolean standsOnLava;
    private final TerrainClassification classification;

    private MobTraversalProfile(Builder builder) {
        this.name = builder.name;
        this.maluses = Collections.unmodifiableMap(new EnumMap<>(builder.maluses));
        this.canPassDoors = builder.canPassDoors;
        this.blockManipulation = builder.blockManipulation;
        this.canFloat = builder.canFloat;
        this.canWalkOverFences = builder.canWalkOverFences;
        this.currentlyOnRail = builder.currentlyOnRail;
        this.prefersJumpToPreferredBlocks = builder.prefersJumpToPreferredBlocks;
        this.standsOnLava = builder.standsOnLava;
        this.classification = byDeclaredEquality(builder.classification);
    }

    private MobTraversalProfile(MobTraversalProfile source,
                                TerrainClassification classification) {
        this.name = source.name;
        this.maluses = source.maluses;
        this.canPassDoors = source.canPassDoors;
        this.blockManipulation = source.blockManipulation;
        this.canFloat = source.canFloat;
        this.canWalkOverFences = source.canWalkOverFences;
        this.currentlyOnRail = source.currentlyOnRail;
        this.prefersJumpToPreferredBlocks = source.prefersJumpToPreferredBlocks;
        this.standsOnLava = source.standsOnLava;
        this.classification = classification;
    }

    private static TerrainClassification byDeclaredEquality(
            TerrainClassification classification) {
        Object key = classification.equalityKey();
        return key == null || key == classification ? classification
                : new KeyedTerrainClassification(classification, key);
    }

    public String name() {
        return name;
    }

    public double malus(TerrainType type) {
        Double override = maluses.get(type);
        return override != null ? override : TerrainCosts.baseline(type);
    }

    public Map<TerrainType, Double> overrides() {
        return maluses;
    }

    public boolean canPassDoors() {
        return canPassDoors;
    }

    public boolean canOpenDoors() {
        return blockManipulation.manipulates(OpenableBlockFamily.DOOR);
    }

    /**
     * Openable block families this mob may toggle. Doors are the only family
     * a baseline mob touches, so anything else is an integration's choice.
     */
    public BlockManipulationCapabilities blockManipulation() {
        return blockManipulation;
    }

    /**
     * View used for physical clearance checks. A closed block this profile may
     * open reads as air, because the planner already routes through it as a
     * {@link TerrainType#WALKABLE_DOOR} node and the follower opens it on
     * arrival. Both halves have to agree on which blocks qualify, so both read
     * {@link BlockTraversalData#isClosedManipulable}.
     *
     * <p>Air rather than the swung panel, deliberately. The two probes fed
     * from here are coarse: the swept-reach one hugs a cell corner along a
     * handful of samples, and the step-up head-room one sits inside the
     * corridor {@code swungPanelBlocks} already sweeps exactly. A panel here
     * refuses crossings the follower makes with room to spare, and catches
     * nothing that test does not.</p>
     */
    public Block.Getter collisionView(Block.Getter blocks) {
        if (blocks == null) throw new IllegalArgumentException("blocks");
        if (!canPassDoors || !blockManipulation.enabled()) return blocks;
        return (x, y, z, condition) -> {
            Block block = blocks.getBlock(x, y, z, condition);
            return BlockTraversalData.isClosedManipulable(
                    block, blockManipulation) ? Block.AIR : block;
        };
    }

    /**
     * The cells a follower opens for one waypoint, in the states it opens them
     * into. Where {@link #collisionView} answers whether a profile may route
     * through a block at all, this answers what it will sweep once it is
     * standing there: an open door, trapdoor, or gate is a shape, not air, and
     * nothing outside the cells the box reaches is opened by standing at the
     * waypoint. A profile that opens nothing gets {@code blocks} itself.
     *
     * <p>{@link ReachedCells} is the single definition of those cells, so
     * {@code BlockManipulator} writes exactly the states this reads.</p>
     */
    public Block.Getter openedFootprintView(
            Block.Getter blocks, Point waypoint, BoundingBox box) {
        if (blocks == null) throw new IllegalArgumentException("blocks");
        if (waypoint == null) throw new IllegalArgumentException("waypoint");
        if (!canPassDoors || !blockManipulation.enabled()) return blocks;
        ReachedCells cells = ReachedCells.at(waypoint, box);
        return (x, y, z, condition) -> {
            Block block = blocks.getBlock(x, y, z, condition);
            return cells.contains(x, y, z)
                    ? BlockTraversalData.afterOpening(block, blockManipulation)
                    : block;
        };
    }

    public boolean canFloat() {
        return canFloat;
    }

    public boolean canWalkOverFences() {
        return canWalkOverFences;
    }

    public boolean currentlyOnRail() {
        return currentlyOnRail;
    }

    /**
     * Treats blocks tagged {@code frog_prefer_jump_to} as open so the search
     * jumps onto them instead of walking around.
     */
    public boolean prefersJumpToPreferredBlocks() {
        return prefersJumpToPreferredBlocks;
    }

    /**
     * Stands on lava the way baseline striders do.
     */
    public boolean standsOnLava() {
        return standsOnLava;
    }

    /**
     * Blocks this profile names itself, asked before the baseline classifier.
     */
    public TerrainClassification classification() {
        return classification;
    }

    /**
     * A search-scoped copy asking {@link #classification()} at most once per
     * distinct block state. A profile without a hook returns itself, so it
     * keeps the identical code path.
     */
    public MobTraversalProfile withMemoizedClassification() {
        if (classification == TerrainClassification.NONE) return this;
        return new MobTraversalProfile(this,
                new MemoizedTerrainClassification(classification));
    }

    /**
     * Value equality over every field an evaluator reads. Modifiers rebuild a
     * profile on each request, so identity comparison would stop the adaptive
     * coordinator from ever matching one actor's own repeated requests.
     */
    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        return object instanceof MobTraversalProfile other
                && canPassDoors == other.canPassDoors
                && canFloat == other.canFloat
                && canWalkOverFences == other.canWalkOverFences
                && currentlyOnRail == other.currentlyOnRail
                && prefersJumpToPreferredBlocks == other.prefersJumpToPreferredBlocks
                && standsOnLava == other.standsOnLava
                && name.equals(other.name)
                && maluses.equals(other.maluses)
                && blockManipulation.equals(other.blockManipulation)
                && classification.equals(other.classification);
    }

    @Override
    public int hashCode() {
        return Objects.hash(name, maluses, blockManipulation, canPassDoors,
                canFloat, canWalkOverFences, currentlyOnRail,
                prefersJumpToPreferredBlocks, standsOnLava, classification);
    }

    public static Builder builder(String name) {
        return new Builder(name);
    }

    public static final class Builder {
        private final String name;
        private final EnumMap<TerrainType, Double> maluses =
                new EnumMap<>(TerrainType.class);
        private boolean canPassDoors = true;
        private BlockManipulationCapabilities blockManipulation =
                BlockManipulationCapabilities.DISABLED;
        private boolean canFloat;
        private boolean canWalkOverFences;
        private boolean currentlyOnRail;
        private boolean prefersJumpToPreferredBlocks;
        private boolean standsOnLava;
        private TerrainClassification classification =
                TerrainClassification.NONE;

        private Builder(String name) {
            if (name == null || name.isBlank()) throw new IllegalArgumentException("name");
            this.name = name;
        }

        public Builder from(MobTraversalProfile profile) {
            this.maluses.putAll(profile.maluses);
            this.canPassDoors = profile.canPassDoors;
            this.blockManipulation = profile.blockManipulation;
            this.canFloat = profile.canFloat;
            this.canWalkOverFences = profile.canWalkOverFences;
            this.currentlyOnRail = profile.currentlyOnRail;
            this.prefersJumpToPreferredBlocks =
                    profile.prefersJumpToPreferredBlocks;
            this.standsOnLava = profile.standsOnLava;
            this.classification = profile.classification;
            return this;
        }

        public Builder malus(TerrainType type, double value) {
            if (type == null || !Double.isFinite(value)) {
                throw new IllegalArgumentException("invalid malus");
            }
            maluses.put(type, value);
            return this;
        }

        public Builder canPassDoors(boolean value) {
            this.canPassDoors = value;
            return this;
        }

        public Builder canOpenDoors(boolean value) {
            this.blockManipulation = blockManipulation.withFamily(
                    OpenableBlockFamily.DOOR, value);
            return this;
        }

        /**
         * Replaces every manipulable family at once. Applied before a later
         * {@link #canOpenDoors(boolean)}, which only refines the door family.
         */
        public Builder blockManipulation(
                BlockManipulationCapabilities capabilities) {
            if (capabilities == null) {
                throw new IllegalArgumentException(
                        "block manipulation capabilities");
            }
            this.blockManipulation = capabilities;
            return this;
        }

        public Builder canFloat(boolean value) {
            this.canFloat = value;
            return this;
        }

        public Builder canWalkOverFences(boolean value) {
            this.canWalkOverFences = value;
            return this;
        }

        public Builder currentlyOnRail(boolean value) {
            this.currentlyOnRail = value;
            return this;
        }

        public Builder prefersJumpToPreferredBlocks(boolean value) {
            this.prefersJumpToPreferredBlocks = value;
            return this;
        }

        public Builder standsOnLava(boolean value) {
            this.standsOnLava = value;
            return this;
        }

        /**
         * Names blocks the baseline classifier does not know. The hook is
         * asked first and falls through wherever it declines.
         */
        public Builder classification(TerrainClassification value) {
            if (value == null) throw new IllegalArgumentException("classification");
            this.classification = value;
            return this;
        }

        public MobTraversalProfile build() {
            return new MobTraversalProfile(this);
        }
    }
}
