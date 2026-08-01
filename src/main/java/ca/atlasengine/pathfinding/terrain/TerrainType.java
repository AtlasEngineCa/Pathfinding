package ca.atlasengine.pathfinding.terrain;

/**
 * What a navigation cell <em>is</em>.
 *
 * <p>A category carries no cost of its own. What a category costs is policy,
 * not identity: it varies by mob, by profile, and by request, so it lives in
 * {@link TerrainCosts} and is layered by {@link MobTraversalProfile}. Keeping
 * the two apart is what lets a profile override a cost without the category
 * needing an opinion, and keeps the defaults in one place to audit.</p>
 *
 * <p><strong>Declaration order is API.</strong> These constants are exported,
 * so their ordinals appear in consumer {@code EnumMap} iteration order, in
 * ordinal-indexed arrays, and in any numeric serialization. Add new categories
 * at the end and never reorder existing ones — the groupings below are
 * documentation only, and deliberately do not follow declaration order.</p>
 *
 * <p>Categories fall into nine roles: the impassable barrier; free space;
 * standing on something notable; fluids; hazards; openables and partial
 * barriers; rails; vegetation; and one size-dependent category. Each constant
 * notes its role.</p>
 */
public enum TerrainType {

    /** Barrier. No route through this cell for any profile. */
    BLOCKED,
    /** Free space. Air with nothing standable beneath it. */
    OPEN,
    /** Free space. Air over solid support: the ordinary ground-search cell. */
    WALKABLE,
    /** Openable. A closed openable this profile may swing and walk through. */
    WALKABLE_DOOR,
    /** Openable. */
    TRAPDOOR,
    /** Hazard. Refused underfoot because a mob sinks into it. */
    POWDER_SNOW,
    /** Standing on. Supported by powder snow rather than sunk in it. */
    ON_TOP_OF_POWDER_SNOW,
    /** Partial barrier. Occupies only part of its cell. */
    FENCE,
    /** Fluid. */
    LAVA,
    /** Fluid. */
    WATER,
    /** Fluid. Water adjacent to a barrier. */
    WATER_BORDER,
    /** Rail. */
    RAIL,
    /** Rail. One a mob not already riding the network may not join. */
    UNPASSABLE_RAIL,
    /** Hazard. Adjacent to fire or lava rather than in it. */
    FIRE_IN_NEIGHBOR,
    /** Hazard. */
    FIRE,
    /** Hazard. Adjacent to a damaging block rather than in it. */
    DAMAGING_IN_NEIGHBOR,
    /** Hazard. */
    DAMAGING,
    /** Openable. */
    DOOR_OPEN,
    /** Openable. Passable only by a profile that can open it. */
    DOOR_WOOD_CLOSED,
    /** Openable. Passable only by a profile that can open it. */
    DOOR_IRON_CLOSED,
    /** Fluid. Air directly above water, reached only by a breaching swimmer. */
    BREACH,
    /** Vegetation. */
    LEAVES,
    /** Hazard. Holds a mob down, grounding its step budget. */
    STICKY_HONEY,
    /** Vegetation. */
    COCOA,
    /** Hazard. Contact damage a mob routes around rather than eats. */
    DAMAGE_CAUTIOUS,
    /** Standing on. Supported by a trapdoor. */
    ON_TOP_OF_TRAPDOOR,
    /**
     * Size-dependent. Assigned when a wide mob's footprint straddles a hazard
     * its anchor cell does not. Caps the penalty so a large mob is discouraged
     * from grazing danger without being walled off by its own width.
     */
    BIG_MOBS_CLOSE_TO_DANGER;

    /**
     * The baseline cost of entering a cell of this category.
     *
     * @deprecated a category no longer owns its cost. Call
     *     {@link TerrainCosts#baseline(TerrainType)} for the same value, or
     *     {@link MobTraversalProfile#malus(TerrainType)} for the cost a
     *     specific mob actually pays, which is what nearly every caller wants.
     *     Retained so existing binaries and sources keep working.
     */
    @Deprecated(since = "10.3.0")
    public double defaultMalus() {
        return TerrainCosts.baseline(this);
    }
}
