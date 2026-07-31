package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.internal.JumpArc;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

/**
 * The solver is the only place that knows the entity's aerodynamics, so it
 * is the only place a jump the entity cannot fly can be refused.
 */
class JumpBallisticsTest {
    /** Minecraft's default living-entity aerodynamics. */
    private static final double GRAVITY = 0.08;
    private static final double DRAG = 0.98;

    @Test
    void aSaturatedBisectionIsReportedRatherThanLaunchedShort() {
        double reachable = JumpBallistics.launchVerticalVelocity(
                0, 8, GRAVITY, DRAG);
        double saturated = JumpBallistics.launchVerticalVelocity(
                0, 8, 2.0, DRAG);

        assertTrue(JumpBallistics.reachesApex(
                reachable, 0, 8, GRAVITY, DRAG));
        assertTrue(JumpBallistics.predictedApex(reachable, GRAVITY, DRAG)
                >= 8, () -> "apex " + JumpBallistics.predictedApex(
                        reachable, GRAVITY, DRAG));
        assertFalse(JumpBallistics.reachesApex(saturated, 0, 8, 2.0, DRAG),
                () -> "a heavy entity reached only "
                        + JumpBallistics.predictedApex(saturated, 2.0, DRAG)
                        + " of the 8 blocks the arc needs");
    }

    @ParameterizedTest
    @ValueSource(doubles = {0, 1, 4, 8})
    void everyApexTheDefaultProfileCanReachAlsoComesBackDown(
            double apexClearance) {
        double velocity = JumpBallistics.launchVerticalVelocity(
                0, apexClearance, GRAVITY, DRAG);

        assertTrue(JumpBallistics.reachesApex(
                velocity, 0, apexClearance, GRAVITY, DRAG));
        assertTrue(JumpBallistics.predictedFlightTicks(
                velocity, 0, GRAVITY, DRAG) > 0);
    }

    /**
     * Without gravity the arc never returns, so there is no flight time to
     * spread the horizontal travel over. Reporting the simulation cap instead
     * stretched a two-block hop across eighty ticks.
     */
    @Test
    void anArcThatNeverDescendsHasNoFlightTime() {
        double velocity = JumpBallistics.launchVerticalVelocity(
                0, 1, 0, DRAG);

        assertEquals(0, JumpBallistics.predictedFlightTicks(
                velocity, 0, 0, DRAG));
        assertEquals(0, JumpBallistics.predictedFlightTicks(
                1, 0, 0, 1));
        assertTrue(JumpBallistics.predictedFlightTicks(
                velocity, 0, GRAVITY, DRAG) > 0,
                "gravity alone decides whether the arc closes");
    }

    /**
     * Every case above launches onto a landing level with the takeoff, the
     * one geometry where the launched apex and the chord the planner swept
     * happened to agree. Nothing looked at a rise, so nothing saw the
     * launch climb {@code rise/2} above the band swept for it.
     */
    @ParameterizedTest
    @CsvSource({
            "1, 1", "2, 1", "3, 1", "3, 2", "4, 2", "8, 1", "16, 4",
            "-1, 1", "-2, 1", "-4, 2", "-16, 1"
    })
    void aLandingRiseLiftsTheLaunchedApexByTheWholeRise(
            double landingRise, double apexClearance) {
        double velocity = JumpBallistics.launchVerticalVelocity(
                landingRise, apexClearance, GRAVITY, DRAG);
        double apex = JumpBallistics.predictedApex(velocity, GRAVITY, DRAG);

        assertEquals(Math.max(0, landingRise) + apexClearance, apex, 1.0e-6,
                "the launch clears the landing and the clearance above it");
        assertEquals(apex, JumpArc.peak(landingRise, apexClearance), 1.0e-9,
                "the arc swept for clearance must want that same apex");
        assertTrue(JumpBallistics.reachesApex(
                velocity, landingRise, apexClearance, GRAVITY, DRAG));
        assertTrue(JumpBallistics.predictedFlightTicks(
                        velocity, landingRise, GRAVITY, DRAG) > 0,
                "a rise the entity can climb must also come back down");
    }

    /**
     * A rise costs launch velocity, so the same clearance over a taller
     * landing is the case that saturates first.
     */
    @ParameterizedTest
    @CsvSource({"0, 8", "4, 8", "8, 8"})
    void aRisingArcNoHeavyEntityCanClimbIsReportedRatherThanLaunchedShort(
            double landingRise, double apexClearance) {
        double reachable = JumpBallistics.launchVerticalVelocity(
                landingRise, apexClearance, GRAVITY, DRAG);
        double saturated = JumpBallistics.launchVerticalVelocity(
                landingRise, apexClearance, 2.0, DRAG);

        assertTrue(JumpBallistics.reachesApex(
                reachable, landingRise, apexClearance, GRAVITY, DRAG));
        assertFalse(JumpBallistics.reachesApex(
                        saturated, landingRise, apexClearance, 2.0, DRAG),
                () -> "a heavy entity reached only "
                        + JumpBallistics.predictedApex(saturated, 2.0, DRAG)
                        + " of the " + (landingRise + apexClearance)
                        + " blocks the arc needs");
    }

    @Test
    void aDropStillClosesWithinTheSimulationWindow() {
        double velocity = JumpBallistics.launchVerticalVelocity(
                -16, 1, GRAVITY, DRAG);

        assertTrue(JumpBallistics.predictedFlightTicks(
                velocity, -16, GRAVITY, DRAG) > 0,
                "the deepest configurable drop must not read as unreachable");
    }
}
