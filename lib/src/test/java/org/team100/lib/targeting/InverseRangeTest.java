package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/** See RangeTest. */
public class InverseRangeTest {
    private static final double DELTA = 0.001;

    @Test
    void test0() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        InverseRange ir = new InverseRange(d, 0, 8, 50);
        FiringParameters p = ir.apply(2.825);
        // direct solution
        assertEquals(0.358, p.elevation(), DELTA);
        assertEquals(0.628, p.tof(), DELTA);
        // indirect solution
        // assertEquals(Math.PI / 4, p.elevation(), DELTA);
        // assertEquals(1.011, p.tof(), DELTA);
    }

}
