package org.team100.lib.config;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;

public class FrictionTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testFriction() {
        // static friction = 2, dynamic friction = 1
        Friction friction = new Friction(log, 2, 1, 0, 1);
        // under the static friction limit, so this is static
        assertEquals(2, friction.frictionFFVolts(0.5), DELTA);
        // over the static friction limit, so sliding
        assertEquals(1, friction.frictionFFVolts(2), DELTA);
        // under the static friction limit, so this is static
        assertEquals(-2, friction.frictionFFVolts(-0.5), DELTA);
        // over the static friction limit, so sliding
        assertEquals(-1, friction.frictionFFVolts(-2), DELTA);
        // want to go negative, get negative
        assertEquals(-2, friction.frictionFFVolts(-0.5), DELTA);
        // moving positive, want to go negative, get negative
        assertEquals(-1, friction.frictionFFVolts(-2), DELTA);
    }

}
