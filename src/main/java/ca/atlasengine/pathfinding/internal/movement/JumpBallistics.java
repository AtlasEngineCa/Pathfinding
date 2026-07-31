package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.internal.JumpArc;

/**
 * Pure ballistic solver for platform jumps. Every method is a function of
 * launch velocity, gravity, and vertical drag alone.
 */
final class JumpBallistics {
    private static final int SIMULATED_TICKS = 80;
    private static final int BISECTION_ITERATIONS = 48;
    private static final double MAX_LAUNCH_VELOCITY = 4;

    private JumpBallistics() {
    }

    static double launchVerticalVelocity(
            double landingRise, double apexClearance,
            double gravity, double verticalDrag) {
        double wantedApex = wantedApex(landingRise, apexClearance);
        double low = 0;
        double high = 0.25;
        while (predictedApex(high, gravity, verticalDrag) < wantedApex
                && high < MAX_LAUNCH_VELOCITY) {
            high *= 2;
        }
        for (int iteration = 0; iteration < BISECTION_ITERATIONS; iteration++) {
            double middle = (low + high) * 0.5;
            if (predictedApex(middle, gravity, verticalDrag) < wantedApex) {
                low = middle;
            } else {
                high = middle;
            }
        }
        return high;
    }

    /** The arc's own peak, so the launch cannot outclimb the swept band. */
    private static double wantedApex(
            double landingRise, double apexClearance) {
        return JumpArc.peak(landingRise, apexClearance);
    }

    /**
     * False when the entity's own gravity and drag cannot lift it that far.
     * {@link #launchVerticalVelocity} saturates silently at
     * {@link #MAX_LAUNCH_VELOCITY}, and a launch that falls short lands in
     * the gap rather than on the platform.
     */
    static boolean reachesApex(
            double initialVelocity, double landingRise, double apexClearance,
            double gravity, double verticalDrag) {
        return predictedApex(initialVelocity, gravity, verticalDrag)
                >= wantedApex(landingRise, apexClearance);
    }

    static double predictedApex(
            double initialVelocity, double gravity, double verticalDrag) {
        double position = 0;
        double velocity = initialVelocity;
        double apex = 0;
        for (int tick = 0; tick < SIMULATED_TICKS; tick++) {
            position += velocity;
            apex = Math.max(apex, position);
            velocity = (velocity - gravity) * verticalDrag;
            if (velocity <= 0 && tick > 0) break;
        }
        return apex;
    }

    /**
     * Ticks until the arc falls back to {@code landingRise}, or {@code 0}
     * when it never does. Without gravity the entity keeps rising, so there
     * is no flight time to spread the horizontal travel over.
     */
    static int predictedFlightTicks(
            double initialVelocity, double landingRise,
            double gravity, double verticalDrag) {
        double position = 0;
        double velocity = initialVelocity;
        boolean descending = false;
        for (int tick = 1; tick <= SIMULATED_TICKS; tick++) {
            position += velocity;
            velocity = (velocity - gravity) * verticalDrag;
            descending |= velocity <= 0;
            if (descending && position <= landingRise) return tick;
        }
        return 0;
    }
}
