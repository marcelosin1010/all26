package org.team100.lib.util;

import org.junit.jupiter.api.Test;

public class IndirectDualMeasurementTest {
    @SuppressWarnings("unused")
    @Test
    void test0() {
        // setup
        int N = 100;
        // m1 and m2 need to be coprime (have no common divisors)
        int m1 = 10;
        int m2 = 11;
        // actual position
        double actualXRotations = 0.5;
        double actualXTeeth = actualXRotations * N;
        double a1 = actualXTeeth % m1;
        double a2 = actualXTeeth % m2;
        IndirectDualMeasurement idm = new IndirectDualMeasurement(N, m1, m2);
        double x = idm.x(a1, a2);
        double measuredXRotations = x / N;
        // assertEquals(actualXRotations, measuredXRotations, 0.001);
    }

}
