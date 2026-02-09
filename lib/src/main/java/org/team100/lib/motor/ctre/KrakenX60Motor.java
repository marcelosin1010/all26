package org.team100.lib.motor.ctre;

import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.util.CanId;

/**
 * Kraken X60 using Phoenix 6.
 * 
 * https://docs.wcproducts.com/welcome/electronics/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance
 */
public class KrakenX60Motor extends Talon6Motor {

    public KrakenX60Motor(
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
        // 12.0 V, 483 A
        return 0.025;
    }

    @Override
    public double kTNm_amp() {
        // 9.39 Nm, 483 A
        return 0.019;
    }

    @Override
    public double kFreeSpeedRPM() {
        return 5800;
    }

    /** Feedforward for swerve drive axis */
    public static SimpleDynamics swerveDriveFF(LoggerFactory log) {
        // TODO: friction here is probably too low.
        // TODO: verify kA
        return new SimpleDynamics(log, 0.004, 0.002);
    }

    public static Friction swerveDriveFriction(LoggerFactory log) {
        // TODO: friction here is probably too low.
        return new Friction(log, 0.26, 0.26, 0.006, 0.5);
    }

    public static SimpleDynamics highFrictionFF(LoggerFactory log) {
        return new SimpleDynamics(log, 0.004, 0.002);
    }

    public static Friction highFriction(LoggerFactory log) {
        return new Friction(log, 0.26, 0.26, 0.006, 0.5);
    }

    public static SimpleDynamics lowFrictionFF(LoggerFactory log) {
        return new SimpleDynamics(log, 0.004, 0.002);
    }
}
