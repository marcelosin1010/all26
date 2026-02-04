package org.team100.lib.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class BareMotorTest {

    @Test
    void test0() {
        BareMotor m = new AbstractBareMotor() {
            @Override
            public double kFreeSpeedRPM() {
                return 114.59;
            }
        };
        assertEquals(1, m.backEMFVoltage(1), 0.001);

    }

}
