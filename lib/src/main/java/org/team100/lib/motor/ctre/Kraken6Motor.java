package org.team100.lib.motor.ctre;

import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.util.CanId;

/**
 * Kraken using Phoenix 6.
 * 
 * TODO: make a separate class for the x44, since its free speed is higher.
 * x60 is 5800 RPM at 12.0 V = 0.124 v-s/rev
 * x44 is 7530 RPM at 12.0 V = 0.096 v-s/rev
 * 
 * @see https://store.ctr-electronics.com/content/datasheet/Motor%20Performance%20Analysis%20Report.pdf
 */
public class Kraken6Motor extends Talon6Motor {

    public Kraken6Motor(
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
        return 0.025;
    }

    @Override
    public double kTNm_amp() {
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
