package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.search.NavigationRequest;
/**
 * Which planner answers one shared-mesh request.
 *
 * <p>Ordinary {@link NavigationSystem#plan(NavigationRequest)} and {@link
 * NavigationSystem#submit(NavigationRequest)} are individual by definition and
 * never consult the mesh, so a strategy is only stated on a {@link
 * ca.atlasengine.pathfinding.adaptive.SharedMeshRequest}.</p>
 */
public enum NavigationStrategy {
    /**
     * Always an ordinary per-entity A* search. The mesh is not consulted, the
     * request joins no region, and its result seeds no later reuse. Use this
     * to keep one caller off the mesh while other callers keep using it.
     */
    INDIVIDUAL_ONLY,
    /**
     * Consults the mesh first and runs an ordinary A* search whenever the
     * mesh cannot answer.
     *
     * <p>This is a preference over the dispatch decision, never over the
     * parity gate: it cannot make a shared plan appear. A shared plan is
     * served only when the region is already promoted and holds a certified
     * ordinary-A* plan for this exact source and destination class that still
     * passes {@link
     * ca.atlasengine.pathfinding.internal.adaptive.NavigationPlanParity}. A missing
     * certificate, a failed parity check, a moved source, an exhausted region,
     * or a disabled mesh silently runs the ordinary search instead. There is
     * no mode that serves an uncertified shared plan.</p>
     */
    PREFER_SHARED
}
