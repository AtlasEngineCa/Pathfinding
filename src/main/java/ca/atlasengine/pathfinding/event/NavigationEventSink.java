package ca.atlasengine.pathfinding.event;

/**
 * Where a controller announces its events.
 *
 * <p>{@link #listening(Class)} is asked before an event is built, so a system
 * nobody listens to allocates nothing on the navigation path.</p>
 */
public interface NavigationEventSink {
    /** A sink that announces nothing and is never listening. */
    NavigationEventSink NONE = new NavigationEventSink() {
        @Override
        public boolean listening(Class<? extends NavigationEvent> type) {
            return false;
        }

        @Override
        public void emit(NavigationEvent event) {
        }
    };

    boolean listening(Class<? extends NavigationEvent> type);

    void emit(NavigationEvent event);
}
