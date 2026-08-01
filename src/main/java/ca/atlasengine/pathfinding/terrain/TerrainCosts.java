package ca.atlasengine.pathfinding.terrain;

import java.util.Arrays;

/**
 * Baseline traversal cost per {@link TerrainType}, in units of one block of
 * travel. A negative cost means impassable; zero means free.
 *
 * <h2>Why these numbers</h2>
 * These defaults provide predictable routing across the built-in profiles.
 * Retuning them is supported and expected, but should normally be done with a
 * {@link MobTraversalProfile} override rather than by editing this table, where
 * a change would silently affect every profile.
 *
 * <p>Costs are resolved in layers, with the most specific configured value
 * replacing the less specific value:</p>
 * <ol>
 *   <li>this baseline,</li>
 *   <li>the profile's per-type overrides
 *       ({@link MobTraversalProfile#malus}),</li>
 *   <li>per-request modifiers
 *       ({@code NavigationModifiers#terrainCosts}).</li>
 * </ol>
 *
 * <p>Only two magnitudes carry meaning beyond ordering. {@code 8} is the
 * "notable deterrent" step used for every discomfort a mob will still accept
 * if the detour is long enough, and {@code 16} is reserved for standing in
 * fire, the one non-fatal cell worth roughly two of those. Everything else is
 * {@code 0} (free), {@code -1} (refused), or a fraction of the deterrent step.
 * Pick from those magnitudes when adding a category rather than inventing a
 * new one.</p>
 */
public final class TerrainCosts {

    /** Refused outright: no profile may route through without an override. */
    private static final double IMPASSABLE = -1;
    /** No penalty beyond the geometric distance of the step. */
    private static final double FREE = 0;
    /** The standard "worth avoiding, not worth refusing" penalty. */
    private static final double DETERRENT = 8;

    private static final double[] BASELINE = createBaseline();

    private TerrainCosts() {
    }

    /**
     * The unmodified cost of entering a cell of this category. Callers wanting
     * the cost a specific mob pays should ask its profile instead.
     */
    public static double baseline(TerrainType type) {
        return BASELINE[type.ordinal()];
    }

    private static double[] createBaseline() {
        double[] costs = new double[TerrainType.values().length];
        Arrays.fill(costs, Double.NaN);

        set(costs, TerrainType.BLOCKED, IMPASSABLE);

        set(costs, TerrainType.OPEN, FREE);
        set(costs, TerrainType.WALKABLE, FREE);
        set(costs, TerrainType.ON_TOP_OF_TRAPDOOR, FREE);
        set(costs, TerrainType.ON_TOP_OF_POWDER_SNOW, FREE);

        // Swimming is accepted but never preferred over dry ground. A dry
        // shoreline cell remains free: water beside a one-cell ledge must not
        // make that ledge look narrower than the entity's collision box.
        set(costs, TerrainType.WATER, DETERRENT);
        set(costs, TerrainType.WATER_BORDER, FREE);
        // Breaching is half a deterrent: surfacing is normal for an air
        // breather, unlike being pushed against a wall.
        set(costs, TerrainType.BREACH, DETERRENT / 2);
        set(costs, TerrainType.LAVA, IMPASSABLE);

        // Standing in fire is the one survivable hazard priced above the
        // deterrent step; being merely next to it costs the step itself.
        set(costs, TerrainType.FIRE, DETERRENT * 2);
        set(costs, TerrainType.FIRE_IN_NEIGHBOR, DETERRENT);
        set(costs, TerrainType.DAMAGING, IMPASSABLE);
        set(costs, TerrainType.DAMAGING_IN_NEIGHBOR, DETERRENT);
        // Free by default: only profiles that actually take contact damage
        // override this, so most mobs walk past sulfur and wither roses.
        set(costs, TerrainType.DAMAGE_CAUTIOUS, FREE);
        // Refused underfoot because a mob sinks; free to stand on top of.
        set(costs, TerrainType.POWDER_SNOW, IMPASSABLE);
        set(costs, TerrainType.STICKY_HONEY, DETERRENT);

        set(costs, TerrainType.DOOR_OPEN, FREE);
        set(costs, TerrainType.WALKABLE_DOOR, FREE);
        set(costs, TerrainType.TRAPDOOR, FREE);
        // Closed doors are refused as terrain. A profile that can open one
        // reaches it as WALKABLE_DOOR instead, so these stay impassable and
        // the capability decides, not the cost.
        set(costs, TerrainType.DOOR_WOOD_CLOSED, IMPASSABLE);
        set(costs, TerrainType.DOOR_IRON_CLOSED, IMPASSABLE);
        set(costs, TerrainType.FENCE, IMPASSABLE);

        set(costs, TerrainType.RAIL, FREE);
        set(costs, TerrainType.UNPASSABLE_RAIL, IMPASSABLE);

        set(costs, TerrainType.LEAVES, IMPASSABLE);
        set(costs, TerrainType.COCOA, FREE);

        // Half a deterrent: enough to prefer a clean route, cheap enough that
        // a wide mob still fits through a corridor that grazes a hazard.
        set(costs, TerrainType.BIG_MOBS_CLOSE_TO_DANGER, DETERRENT / 2);

        for (double cost : costs) {
            if (Double.isNaN(cost)) {
                throw new IllegalStateException(
                        "baseline cost missing for a terrain category");
            }
        }
        return costs;
    }

    private static void set(double[] costs, TerrainType type, double cost) {
        costs[type.ordinal()] = cost;
    }
}
