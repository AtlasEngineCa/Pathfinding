package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.internal.JumpArc;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import java.util.Locale;

import static org.junit.jupiter.api.Assertions.*;

/**
 * The planner sweeps a curve; the follower flies a simulation. Matching the
 * two peaks is not enough, because drag makes the real trajectory an
 * asymmetric curve that can sit above a symmetric one anywhere, so the
 * envelope is compared against the simulation at every sample of every
 * flight rather than at the top of it.
 */
class JumpArcEnvelopeTest {
    /** Bounds {@link JumpArc#headroom} is written for. */
    private static final double[] GRAVITIES = {
            0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10, 0.12, 0.14};
    private static final double[] DRAGS = {
            0.90, 0.92, 0.94, 0.95, 0.96, 0.97, 0.98, 0.99, 0.995, 1.0};
    private static final double[] RISES = {
            -4, -3, -2, -1, -0.5, 0, 0.5, 1, 2, 3, 4, 6, 8, 12, 16};
    private static final double[] APEXES = {
            0, 0.05, 0.25, 0.5, 1, 1.5, 2, 3, 4, 6, 8, 12, 16};
    private static final double[] SPANS = {2, 3, 4, 6, 8, 16, 32};

    /** Minecraft's default living-entity aerodynamics. */
    private static final double GRAVITY = 0.08;
    private static final double DRAG = 0.98;

    @Test
    void theSweptEnvelopeStaysAboveEveryFlightItIsSweptFor() {
        double worst = Double.NEGATIVE_INFINITY;
        String worstCase = "";
        long flights = 0;
        for (double rise : RISES) {
            for (double apex : APEXES) {
                double headroom = JumpArc.headroom(rise, apex);
                for (double gravity : GRAVITIES) {
                    for (double drag : DRAGS) {
                        double[] flight = flight(rise, apex, gravity, drag);
                        if (flight == null) continue;
                        flights++;
                        for (double span : SPANS) {
                            for (int tick = 0; tick < flight.length; tick++) {
                                double t = (double) tick
                                        / (flight.length - 1);
                                double deviation = flight[tick] - headroom
                                        - swept(rise, apex, span, t);
                                if (deviation <= worst) continue;
                                worst = deviation;
                                worstCase = String.format(Locale.ROOT,
                                        "rise=%.1f apex=%.2f span=%.0f "
                                                + "gravity=%.2f drag=%.3f "
                                                + "t=%.3f",
                                        rise, apex, span, gravity, drag, t);
                            }
                        }
                    }
                }
            }
        }

        String detail = worstCase;
        double deviation = worst;
        long covered = flights;
        assertTrue(covered > 5_000, () -> "only " + covered + " flights");
        assertTrue(deviation <= 0, () -> "the flight rose " + deviation
                + " above the swept envelope at " + detail);
    }

    /**
     * The band the old chord left unswept, measured. Every upward jump flew
     * through it, and nothing in the suite looked above the chord.
     */
    @ParameterizedTest
    @CsvSource({"1, 1, 0.4375", "2, 1, 0.7500", "3, 2, 1.2188"})
    void theChordItReplacedSatBelowTheFlightItWasSweptFor(
            double rise, double apex, double expectedShortfall) {
        double chordPeak = Double.NEGATIVE_INFINITY;
        for (int sample = 0; sample <= 1_000; sample++) {
            double t = sample / 1_000.0;
            chordPeak = Math.max(chordPeak,
                    rise * t + 4 * apex * t * (1 - t));
        }

        assertEquals(expectedShortfall,
                JumpArc.peak(rise, apex) - chordPeak, 1.0e-3,
                "the chord's own peak fell short of the launched one");
        assertTrue(JumpArc.headroom(rise, apex) > 0);
    }

    @ParameterizedTest
    @CsvSource({"0, 1", "1, 1", "2, 1", "3, 2", "-2, 1", "-4, 0.05"})
    void theArcEndsOnTheLandingAndPeaksWhereTheLaunchDoes(
            double rise, double apex) {
        double peak = Double.NEGATIVE_INFINITY;
        for (int sample = 0; sample <= 4_000; sample++) {
            peak = Math.max(peak,
                    JumpArc.height(rise, apex, sample / 4_000.0));
        }

        assertEquals(0, JumpArc.height(rise, apex, 0), 1.0e-12);
        assertEquals(rise, JumpArc.height(rise, apex, 1), 1.0e-12);
        assertEquals(JumpArc.peak(rise, apex), peak, 1.0e-6);
        assertEquals(JumpArc.peak(rise, apex),
                JumpBallistics.predictedApex(
                        JumpBallistics.launchVerticalVelocity(
                                rise, apex, GRAVITY, DRAG), GRAVITY, DRAG),
                1.0e-6, "the arc and the launch must want the same peak");
    }

    /** A level arc is the chord it always was, so nothing level moves. */
    @ParameterizedTest
    @CsvSource({"0.05", "0.25", "1", "2", "8"})
    void aLevelArcIsUnchanged(double apex) {
        for (int sample = 0; sample <= 200; sample++) {
            double t = sample / 200.0;
            assertEquals(4 * apex * t * (1 - t),
                    JumpArc.height(0, apex, t), 1.0e-12,
                    () -> "t=" + t);
        }
    }

    /** Ticked heights of the arc the executor actually launches. */
    private static double[] flight(
            double rise, double apex, double gravity, double drag) {
        double launch = JumpBallistics.launchVerticalVelocity(
                rise, apex, gravity, drag);
        if (!JumpBallistics.reachesApex(launch, rise, apex, gravity, drag)) {
            return null;
        }
        int ticks = JumpBallistics.predictedFlightTicks(
                launch, rise, gravity, drag);
        if (ticks <= 0) return null;
        // The executor spreads the horizontal travel over one tick less.
        int spread = Math.max(2, ticks - 1);
        double[] heights = new double[spread + 1];
        double height = 0;
        double velocity = launch;
        for (int tick = 1; tick <= spread; tick++) {
            height += velocity;
            velocity = (velocity - gravity) * drag;
            heights[tick] = height;
        }
        return heights;
    }

    /**
     * The envelope the sweep really tests: a box travels the chord between
     * two arc samples, not the arc itself.
     */
    private static double swept(
            double rise, double apex, double span, double t) {
        int samples = Math.max(8, (int) Math.ceil(span * 16));
        double scaled = t * samples;
        int previous = Math.min(samples - 1, (int) Math.floor(scaled));
        double from = JumpArc.height(rise, apex, (double) previous / samples);
        double to = JumpArc.height(rise, apex, (previous + 1.0) / samples);
        return from + (to - from) * (scaled - previous);
    }
}
