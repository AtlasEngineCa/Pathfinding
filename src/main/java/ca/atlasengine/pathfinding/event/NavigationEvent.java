package ca.atlasengine.pathfinding.event;

import ca.atlasengine.pathfinding.NavigationSystem;import net.minestom.server.event.Event;

/**
 * Anything a {@link ca.atlasengine.pathfinding.NavigationSystem} announces.
 *
 * <p>Every event is called on the thread that ticked the controller, so a
 * listener may touch the entity directly. Searches themselves run on worker
 * threads and never call listeners; a completed search is announced when the
 * owning controller next ticks and adopts it.</p>
 */
public interface NavigationEvent extends Event {
}
