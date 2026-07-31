package ca.atlasengine.pathfinding.internal.movement;

import net.minestom.server.coordinate.Point;

/**
 * Watchdogs for a follower that stops making headway: the per-waypoint
 * timeout and the long-window travel stall.
 */
public final class ProgressMonitor {
    /** Ticks a waypoint may take relative to its distance-derived budget. */
    private static final long WAYPOINT_TIMEOUT_SLACK = 3L;
    /** Window over which lack of travel is treated as a stall. */
    private static final int STALL_WINDOW_TICKS = 100;
    /** Fraction of the achievable travel required within the stall window. */
    private static final double STALL_PROGRESS_FRACTION = 0.25;

    private final double movementPerTick;

    private int lastProgressTick;
    private Point lastProgressPosition;
    private int waypointTicks;
    private int waypointTimeoutLimit;

    public ProgressMonitor(double movementPerTick) {
        this.movementPerTick = movementPerTick;
    }

    public void resetWaypoint() {
        waypointTicks = 0;
        waypointTimeoutLimit = 0;
    }

    public void resetProgress(Point position, int tick) {
        lastProgressPosition = position;
        lastProgressTick = tick;
    }

    public void armWaypointTimeout(Point position, Point waypoint) {
        if (waypointTimeoutLimit == 0) {
            waypointTimeoutLimit = Math.max(1,
                    (int) Math.ceil(position.distance(waypoint) / movementPerTick));
        }
        waypointTicks++;
    }

    public boolean waypointTimedOut() {
        return waypointTicks > waypointTimeoutLimit * WAYPOINT_TIMEOUT_SLACK;
    }

    public boolean stalled(Point position, int tick) {
        if (lastProgressPosition == null) {
            lastProgressPosition = position;
            return false;
        }
        if (tick - lastProgressTick <= STALL_WINDOW_TICKS) return false;
        double effectiveSpeed = movementPerTick >= 1
                ? movementPerTick : movementPerTick * movementPerTick;
        double requiredDistance = effectiveSpeed * STALL_WINDOW_TICKS
                * STALL_PROGRESS_FRACTION;
        boolean stalled =
                position.distance(lastProgressPosition) < requiredDistance;
        lastProgressPosition = position;
        lastProgressTick = tick;
        return stalled;
    }
}
