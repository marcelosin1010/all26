package org.team100.lib.config;

import org.team100.lib.logging.LoggerFactory;

/**
 * The REV PID units are as follows:
 * 
 * position
 * P = duty cycle per revolution, start with 1.
 * I = duty cycle per revolution * ms
 * D = duty cycle per revolution/ms
 * 
 * velocity
 * P = duty cycle per RPM, start with 0.0002; rev example is 0.00005
 * I = duty cycle per RPM*ms
 * D = duty cycle per RPM/ms
 * 
 * @see https://docs.revrobotics.com/revlib/spark/closed-loop/units
 * @see https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
 */
public class RevPIDConstants extends PIDConstants {

    RevPIDConstants(LoggerFactory log,
            double positionP, double positionI, double positionD,
            double velocityP, double velocityI, double velocityD) {
        super(log,
                positionP, positionI, positionD,
                velocityP, velocityI, velocityD);
    }

    /**
     * @param log For mutable.
     * @param p   Duty cycle per revolution.
     */
    public static RevPIDConstants makePositionPID(
            LoggerFactory log, double p) {
        return new RevPIDConstants(log, p, 0, 0, 0, 0, 0);
    }

    /**
     * WARNING! REV velocity control is probably not what you want.
     * 
     * @param log For mutable.
     * @param p   Duty cycle per RPM.
     */
    public static RevPIDConstants makeVelocityPID(
            LoggerFactory log, double p) {
        return new RevPIDConstants(log, 0, 0, 0, p, 0, 0);
    }

    /** Zero is for when you're not using the motor's PID controller */
    public static RevPIDConstants zero(LoggerFactory log) {
        return new RevPIDConstants(log, 0, 0, 0, 0, 0, 0);
    }

}
