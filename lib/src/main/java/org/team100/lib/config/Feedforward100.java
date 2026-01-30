package org.team100.lib.config;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.tuning.Mutable;

/**
 * Feedforward model with three constants, and friction.
 * 
 * @see https://en.wikipedia.org/wiki/Motor_constants
 * @see {@link edu.wpi.first.math.controller.SimpleMotorFeedforward} which uses
 *      a similar model, and also a discrete one which is more accurate.
 * @see {@link org.team100.lib.config.FeedforwardTest} which compares the
 *      models.
 */
public class Feedforward100 {
    private final Mutable kV;
    private final Mutable kA;
    private final Mutable kD;
    private final Friction friction;

    /**
     * @param kV       Back-EMF. Voltage to maintain speed against the back-EMF of
     *                 the motor. V = kV * omega, so kV units are volt-sec/rev.
     *                 The value is an intrinsic property of the motor.
     * @param kA       Acceleration. Voltage to produce acceleration of the motor
     *                 shaft. V = kA * alpha, so kA units are volt-sec^2/rev.
     *                 This reflects the motor torque and mechanism inertia. Torque
     *                 is proportional to current, which is proportional to (net)
     *                 voltage. The value will depend on the inertia of the
     *                 mechanism.
     * @param kD       Deceleration. like kA but when the motor is braking, i.e.
     *                 acceleration is opposite to the current speed. Motors
     *                 typically decelerate ("plugging") much better than they
     *                 accelerate ("motoring").
     * @param friction Models static, dynamic, and viscous friction.
     */
    public Feedforward100(
            LoggerFactory log,
            double kV,
            double kA,
            double kD,
            Friction friction) {
        LoggerFactory ffLog = log.type(this);
        this.kV = new Mutable(ffLog, "kV", kV);
        this.kA = new Mutable(ffLog, "kA", kA);
        this.kD = new Mutable(ffLog, "kD", kD);
        this.friction = friction;
    }

    /**
     * Voltage to maintain the specified velocity.
     * 
     * @param motorRev_S setpoint speed
     */
    public double velocityFFVolts(double motorRev_S) {
        return kV.getAsDouble() * motorRev_S;
    }

    /**
     * Voltage to produce the specified acceleration.
     * 
     * Uses kA when speed and accel are in the same direction.
     * Uses kD when speed and accel are opposite.
     * 
     * @param motorRev_S   setpoint speed
     * @param motorRev_S_S setpoint acceleration
     */
    public double accelFFVolts(double motorRev_S, double motorRev_S_S) {
        if (motorRev_S >= 0) {
            // moving forward
            if (motorRev_S_S >= 0) {
                // faster
                return kA.getAsDouble() * motorRev_S_S;
            } else {
                // slower
                return kD.getAsDouble() * motorRev_S_S;
            }
        } else {
            // moving backward
            if (motorRev_S_S < 0) {
                // faster
                return kA.getAsDouble() * motorRev_S_S;
            } else {
                // slower
                return kD.getAsDouble() * motorRev_S_S;
            }
        }
    }

    /**
     * Voltage to balance friction (i.e. this has the same sign as the supplied
     * speed).
     * 
     * @param motorRev_S setpoint speed
     */
    public double frictionFFVolts(double motorRev_S) {
        return friction.frictionFFVolts(motorRev_S);
    }

    public static Feedforward100 zero(LoggerFactory log) {
        return new Feedforward100(log, 0, 0, 0,
                new Friction(log, 0, 0, 0.0, 0));
    }

    public static Feedforward100 test(LoggerFactory log) {
        return new Feedforward100(log, 0.100, 0.100, 0.100,
                new Friction(log, 0.100, 0.100, 0.0, 0.1));
    }
}
