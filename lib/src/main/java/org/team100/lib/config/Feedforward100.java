package org.team100.lib.config;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.tuning.Mutable;

/**
 * Feedforward for acceleration, deceleration, and friction.
 * 
 * @see https://en.wikipedia.org/wiki/Motor_constants
 * @see {@link edu.wpi.first.math.controller.SimpleMotorFeedforward} which uses
 *      a similar model, and also a discrete one which is more accurate.
 * @see {@link org.team100.lib.config.FeedforwardTest} which compares the
 *      models.
 */
public class Feedforward100 {
    /** volt-sec^2/rad */
    private final Mutable kA;
    /** volt-sec^2/rad */
    private final Mutable kD;
    private final Friction friction;

    /**
     * @param kA           Acceleration. Voltage to produce acceleration of the
     *                     motor shaft. V = kA * alpha, so kA units are
     *                     volt-sec^2/rad. This reflects the motor torque and
     *                     mechanism inertia. Torque is proportional to current,
     *                     which is proportional to (net) voltage. The value will
     *                     depend on the inertia of the mechanism.
     * @param kD           Deceleration. like kA but when the motor is braking, i.e.
     *                     acceleration is opposite to the current speed. Motors
     *                     typically decelerate ("plugging") much better than they
     *                     accelerate ("motoring").
     * @param friction     Models static, dynamic, and viscous friction.
     */
    public Feedforward100(
            LoggerFactory log,
            double kA,
            double kD,
            Friction friction) {
        LoggerFactory ffLog = log.type(this);
        this.kA = new Mutable(ffLog, "kA", kA);
        this.kD = new Mutable(ffLog, "kD", kD);
        this.friction = friction;
    }

    /**
     * Voltage to produce the specified acceleration.
     * 
     * Uses kA when speed and accel are in the same direction.
     * Uses kD when speed and accel are opposite.
     * 
     * @param motorRad_S   setpoint speed, rad/s
     * @param motorRad_S_S setpoint acceleration, rad/s^2
     */
    public double accelFFVolts(double motorRad_S, double motorRad_S_S) {
        if (motorRad_S >= 0) {
            // moving forward
            if (motorRad_S_S >= 0) {
                // faster
                return kA.getAsDouble() * motorRad_S_S;
            } else {
                // slower
                return kD.getAsDouble() * motorRad_S_S;
            }
        } else {
            // moving backward
            if (motorRad_S_S < 0) {
                // faster
                return kA.getAsDouble() * motorRad_S_S;
            } else {
                // slower
                return kD.getAsDouble() * motorRad_S_S;
            }
        }
    }

    /**
     * Voltage to balance friction (i.e. this has the same sign as the supplied
     * speed).
     * 
     * Includes viscous friction (proportional to speed), dynamic friction (constant
     * while moving), and static friction (constant while almost stopped).
     * 
     * @param motorRad_S setpoint speed rad/s
     */
    public double frictionFFVolts(double motorRad_S) {
        return friction.frictionFFVolts(motorRad_S);
    }

    public static Feedforward100 zero(LoggerFactory log) {
        return new Feedforward100(log, 0, 0,
                new Friction(log, 0, 0, 0, 0));
    }

    public static Feedforward100 test(LoggerFactory log) {
        return new Feedforward100(log, 0.100, 0.100,
                new Friction(log, 0.100, 0.100, 0.0, 0.1));
    }
}
