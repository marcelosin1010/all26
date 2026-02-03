package org.team100.lib.config;

import org.team100.lib.logging.LoggerFactory;

/**
 * CTRE PID units depend on the output type. Because we use "voltage" control
 * types ("PositionVoltage" and "VelocityVoltage"), our output type is volts.
 * 
 * position
 * P = volts per revolution, start with 1.
 * I = volts per revolution * sec
 * D = volts per rev/sec
 * 
 * velocity
 * P = volts per rev/sec (volt-sec/rev), start with 0.01
 * I = volts per revolution
 * D = volts per rev/s^2
 * 
 * @see https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html
 */
public class PhoenixPIDConstants extends PIDConstants {

    PhoenixPIDConstants(LoggerFactory log,
            double positionP, double positionI, double positionD,
            double velocityP, double velocityI, double velocityD) {
        super(log,
                positionP, positionI, positionD,
                velocityP, velocityI, velocityD);
    }

    /**
     * @param log For mutable.
     * @param p   volts/rev
     */
    public static PhoenixPIDConstants makePositionPID(
            LoggerFactory log, double p) {
        return new PhoenixPIDConstants(log, p, 0, 0, 0, 0, 0);
    }

    /**
     * @param log For mutable.
     * @param p   volt-sec/rev
     */
    public static PhoenixPIDConstants makeVelocityPID(
            LoggerFactory log, double p) {
        return new PhoenixPIDConstants(log, 0, 0, 0, p, 0, 0);
    }

    /** Zero is for when you're not using the motor's PID controller */
    public static PhoenixPIDConstants zero(LoggerFactory log) {
        return new PhoenixPIDConstants(log, 0, 0, 0, 0, 0, 0);
    }

}
