package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

public class VariableR1Test {
    private static final boolean DEBUG = true;
    private static final double DELTA = 0.001;

    @Test
    void testAdd0() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 1);
        VariableR1 c = VariableR1.add(a, b);
        assertEquals(1, c.mean(), DELTA);
        assertEquals(2, c.variance(), DELTA);
    }

    // MIXTURE MODEL
    //
    // Never becomes more confident than the measurement

    @Test
    void testFuse1a() {
        // Fuse with self
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(0, 1);
        VariableR1 c = VariableR1.fuse1(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Self-fusion has no effect: this isn't like "more evidence".
        assertEquals(1, c.variance(), DELTA);
    }

    @Test
    void testFuse1b() {
        VariableR1 a = new VariableR1(0, 1);
        // "don't know" variance
        VariableR1 b = new VariableR1(1, 100);
        VariableR1 c = VariableR1.fuse1(a, b);
        assertEquals(0.01, c.mean(), DELTA);
        // This is trying to *include* (some of) the high variance
        // instead of ignoring it.
        assertEquals(1.99, c.variance(), DELTA);
    }

    @Test
    void testFuse1c() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 1);
        VariableR1 c = VariableR1.fuse1(a, b);
        // Equal variance: mean is in the middle.
        assertEquals(0.5, c.mean(), DELTA);
        // Dispersion of the mean adds 0.25
        assertEquals(1.25, c.variance(), DELTA);
    }

    @Test
    void testFuse1d() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 0.1);
        VariableR1 c = VariableR1.fuse1(a, b);
        // Equal variance: mean is in the middle.
        assertEquals(0.909, c.mean(), DELTA);
        // Combination of very-confident update but
        // different mean
        assertEquals(0.264, c.variance(), DELTA);
    }

    // INVERSE-VARIANCE WEIGHTING
    //
    // Becomes overconfident, ignores mean dispersion.

    @Test
    void testFuse2a() {
        // Fuse with self
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(0, 1);
        VariableR1 c = VariableR1.fuse2(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Self-fusion increases confidence (too much)
        assertEquals(0.5, c.variance(), DELTA);
    }

    @Test
    void testFuse2b() {
        VariableR1 a = new VariableR1(0, 1);
        // "don't know" variance
        VariableR1 b = new VariableR1(1, 100);
        VariableR1 c = VariableR1.fuse2(a, b);
        // Mostly ignores the uncertain input
        assertEquals(0.01, c.mean(), DELTA);
        // Note the variance here is about the same
        // but *less* than the previous, which is wrong.
        assertEquals(0.99, c.variance(), DELTA);
    }

    @Test
    void testFuse2c() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 1);
        VariableR1 c = VariableR1.fuse2(a, b);
        assertEquals(0.5, c.mean(), DELTA);
        // variance ignores mean dispersion
        assertEquals(0.5, c.variance(), DELTA);
    }

    // COVARIANCE INFLATION
    //
    // Inverse-variance weighting with mean dispersion

    @Test
    void testFuse3a() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(0, 1);
        VariableR1 c = VariableR1.fuse3(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Overconfident.
        assertEquals(0.5, c.variance(), DELTA);
    }

    @Test
    void testFuse3a1() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(0, 100);
        VariableR1 c = VariableR1.fuse3(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Ignores the high variance
        assertEquals(0.99, c.variance(), DELTA);
    }

    @Test
    void testFuse3a2() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(0, 0.1);
        VariableR1 c = VariableR1.fuse3(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Minimum variance applies
        assertEquals(0.091, c.variance(), DELTA);
    }

    @Test
    void testFuse3b() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 100);
        VariableR1 c = VariableR1.fuse3(a, b);
        // Ignores the higher variance
        assertEquals(0.01, c.mean(), DELTA);
        // Ignores the higher variance
        assertEquals(0.99, c.variance(), DELTA);
    }

    @Test
    void testFuse3c() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 1);
        VariableR1 c = VariableR1.fuse3(a, b);
        // Equal variance -> expectation in the middle
        assertEquals(0.5, c.mean(), DELTA);
        // Respects mean dispersion (a little)
        assertEquals(0.505, c.variance(), DELTA);
    }

    @Test
    void testFuse3d() {
        VariableR1 a = new VariableR1(0, 0.0000001);
        VariableR1 b = new VariableR1(0, 0.00000001);
        VariableR1 c = VariableR1.fuse3(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Minimum applies.
        assertEquals(9e-6, c.variance(), 1e-6);
    }

    /**
     * Output is plotted here
     * https://docs.google.com/spreadsheets/d/1DmHL1UDd6vngmr-5_9fNHg2xLC4TEVWTN2nHZBOnje0/edit?gid=1604242948#gid=1604242948
     */
    @Test
    void testFuse3e() {
        Random random = new Random();
        // note non-zero initial mean
        // note not-huge initial variance (1 cm stddev)
        // which might be
        VariableR1 state = new VariableR1(0.3, 0.0001);
        double cameraStdDev = 0.03; // typical camera stddev
        double cameraVariance = cameraStdDev * cameraStdDev;
        if (DEBUG)
            System.out.println("t, camera, state, state stddev");
        for (double t = 0; t < 10; t += 0.02) {
            double cameraEstimate;
            if (t < 3)
                cameraEstimate = 0.0;
            else if (t < 6)
                cameraEstimate = 0.3;
            else
                cameraEstimate = 0.0;
            VariableR1 camera = new VariableR1(random.nextGaussian() * cameraStdDev + cameraEstimate, cameraVariance);
            // state prior to update
            if (DEBUG)
                System.out.printf("%f, %f, %f, %f\n",
                        t, camera.mean(), state.mean(), Math.sqrt(state.variance()));
            // do the update
            state = VariableR1.fuse3(state, camera);
        }
    }

    // BAYESIAN
    //
    // Identical to inverse-variance weighting.

    @Test
    void testFuse4a() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(0, 1);
        VariableR1 c = VariableR1.fuse4(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Overconfident.
        assertEquals(0.5, c.variance(), DELTA);
    }

    @Test
    void testFuse4b() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 100);
        VariableR1 c = VariableR1.fuse4(a, b);
        assertEquals(0.01, c.mean(), DELTA);
        // Adopts the lower variance.
        assertEquals(0.99, c.variance(), DELTA);
    }

    @Test
    void testFuse4c() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 1);
        VariableR1 c = VariableR1.fuse4(a, b);
        assertEquals(0.5, c.mean(), DELTA);
        // Ignores mean dispersion :-(
        assertEquals(0.5, c.variance(), DELTA);
    }

    @Test
    void testWeight0() {
        // Same variance.
        double varA = 1;
        double varB = 1;
        double w = VariableR1.weight(varA, varB);
        // Equal weights.
        assertEquals(0.5, w, DELTA);
    }

}
