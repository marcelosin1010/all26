package org.team100.lib.motor.ctre;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PhoenixPIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
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
            NeutralMode neutral,
            MotorPhase motorPhase,
            double supplyLimit,
            double statorLimit,
            PhoenixPIDConstants pid,
            Feedforward100 ff) {
        super(parent, canId, neutral, motorPhase, supplyLimit, statorLimit, pid, ff);
    }

    @Override
    public double kROhms() {
        return 0.03;
    }

    @Override
    public double kTNm_amp() {
        return 0.018;
    }

    /** Generic feedforward with considerable friction */
    public static Feedforward100 ff(LoggerFactory log) {
        // there's very little friction in the direct drive
        return new Feedforward100(log, 6079, 0.000, 0.000,
                new Friction(log, 0.900, 0.900, 0.0, 0.5));
    }

    /** Feedforward with less friction */
    public static Feedforward100 ff2(LoggerFactory log) {
        return new Feedforward100(log, 6079, 0.001, 0.001,
                new Friction(log, 0.100, 0.065, 0.0, 0.5));
    }

    /** Feedforward for swerve drive motor */
    public static Feedforward100 swerveDriveFF(LoggerFactory log) {
        // there's a little bit of viscous friction here.
        // TODO: verify kA
        return new Feedforward100(log, 6079, 0.003, 0.003,
                new Friction(log, 0.260, 0.260, 0.002, 0.5));
    }

    /**
     * Voltage feedforward for steering motors in air.
     * 9/24/04 Tuned in air, not on carpet, so friction is too low.
     */
    public static Feedforward100 swerveSteerFF(LoggerFactory log) {
        // TODO: the friction here is probably too low
        // TODO: verify kA
        return new Feedforward100(log, 6079, 0.002, 0.002,
                new Friction(log, 0.100, 0.100, 0.005, 0.5));
    }

}
