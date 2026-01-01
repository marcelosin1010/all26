package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class QuadraticInterpolatorTest {
    private static final boolean DEBUG = true;

    /** Fits linear exactly. */
    @Test
    void testLinear() {
        for (double x = 0; x < 2; x += 0.1) {
            double y = QuadraticInterpolator.interpolate(0, 0, 1, 1, 2, 2, x);
            assertEquals(x, y, 1e-9);
        }
    }

    /** Fits the quadratic exactly. */
    @Test
    void testQuadratic() {
        for (double x = 0; x < 2; x += 0.1) {
            double y = QuadraticInterpolator.interpolate(0, 0, 1, 1, 2, 4, x);
            assertEquals(x * x, y, 1e-9);
        }
    }

    /** Doesn't fit the sqrt function very well. */
    @Test
    void testSqrt() {
        if (DEBUG)
            System.out.println("x, actual y, estimate y");
        for (double x = 0; x < 2; x += 0.1) {
            double y = QuadraticInterpolator.interpolate(0, 0, 1, 1, 2, Math.sqrt(2), x);
            if (DEBUG)
                System.out.printf("%f, %f, %f\n", x, Math.sqrt(x), y);
        }
    }

}
