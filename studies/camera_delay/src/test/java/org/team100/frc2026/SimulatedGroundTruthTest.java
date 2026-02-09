package org.team100.frc2026;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;

public class SimulatedGroundTruthTest {
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testStartStop() {
        SimulatedGroundTruth t = new SimulatedGroundTruth(log);

        // Initial position is zero.
        assertEquals(0.00, t.apply(0.00).getRadians(), 0.001);

        // Position does not change with time.
        assertEquals(0.00, t.apply(0.02).getRadians(), 0.001);

        // Start movement at t=0.02.
        t.start(0.02);

        // Now position changes with time.
        assertEquals(0.02, t.apply(0.04).getRadians(), 0.001);
        assertEquals(0.04, t.apply(0.06).getRadians(), 0.001);

        // Stop movement at t=0.06 (position as above, 0.04).
        t.stop(0.06);

        // Position does not change with time.
        assertEquals(0.04, t.apply(0.08).getRadians(), 0.001);
    }

}
