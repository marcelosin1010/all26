package org.team100.lib.motor.ctre;

import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.util.CanId;

/**
 * Falcon 500 using Phoenix 6.
 * 
 * @see https://store.ctr-electronics.com/content/datasheet/Motor%20Performance%20Analysis%20Report.pdf
 */
public class Falcon6Motor extends Talon6Motor {

    public Falcon6Motor(
            LoggerFactory parent,
            CanId canId,
            NeutralMode100 neutral,
            MotorPhase motorPhase,
            double supplyLimit,
            double statorLimit,
            SimpleDynamics ff,
            Friction friction,
            PIDConstants pid) {
        super(parent, canId, neutral, motorPhase, supplyLimit, statorLimit, ff, friction, pid);
    }

    @Override
    public double kROhms() {
        return 0.03;
    }

    @Override
    public double kTNm_amp() {
        return 0.018;
    }

    @Override
    public double kFreeSpeedRPM() {
        return 6079;
    }

    /** Generic feedforward with considerable friction */
    public static SimpleDynamics ff(LoggerFactory log) {
        // there's very little friction in the direct drive
        return new SimpleDynamics(log, 0.000, 0.000);
    }

    public static Friction friction(LoggerFactory log) {
        // there's very little friction in the direct drive
        return new Friction(log, 0.900, 0.900, 0.0, 0.5);
    }

    /** Feedforward with less friction */
    public static SimpleDynamics ff2(LoggerFactory log) {
        return new SimpleDynamics(log, 0.001, 0.001);
    }

    public static Friction friction2(LoggerFactory log) {
        return new Friction(log, 0.100, 0.065, 0.0, 0.5);
    }

    /** Feedforward for swerve drive motor */
    public static SimpleDynamics swerveDriveFF(LoggerFactory log) {
        // there's a little bit of viscous friction here.
        // TODO: verify kA
        return new SimpleDynamics(log, 0.003, 0.003);
    }

    public static Friction swerveDriveFriction(LoggerFactory log) {
        // there's a little bit of viscous friction here.
        return new Friction(log, 0.260, 0.260, 0.002, 0.5);
    }

    /**
     * Voltage feedforward for steering motors in air.
     * 9/24/04 Tuned in air, not on carpet, so friction is too low.
     */
    public static SimpleDynamics swerveSteerFF(LoggerFactory log) {
        // TODO: the friction here is probably too low
        // TODO: verify kA
        return new SimpleDynamics(log, 0.002, 0.002);
    }

    public static Friction swerveSteerFriction(LoggerFactory log) {
        // TODO: the friction here is probably too low
        // TODO: verify kA
        return new Friction(log, 0.100, 0.100, 0.005, 0.5);
    }

}
