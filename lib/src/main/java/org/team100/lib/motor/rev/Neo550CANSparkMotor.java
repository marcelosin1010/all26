package org.team100.lib.motor.rev;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.util.CanId;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/**
 * Neo550 motor.
 * 
 * @see https://www.revrobotics.com/rev-21-1651/
 */
public class Neo550CANSparkMotor extends CANSparkMotor {
    public Neo550CANSparkMotor(
            LoggerFactory parent,
            CanId canId,
            NeutralMode neutral,
            MotorPhase motorPhase,
            int statorCurrentLimit,
            Feedforward100 ff,
            PIDConstants pid) {
        super(parent, new SparkMax(canId.id, MotorType.kBrushless),
                neutral, motorPhase, statorCurrentLimit, ff, pid);
    }

    /** Real or simulated depending on identity */
    public static BareMotor get(
            LoggerFactory log, CanId can, MotorPhase phase, int statorLimit,
            Feedforward100 ff, PIDConstants pid) {
        return switch (Identity.instance) {
            case BLANK ->
                new SimulatedBareMotor(log, 600);
            default -> new Neo550CANSparkMotor(
                    log, can, NeutralMode.BRAKE, phase, statorLimit, ff, pid);
        };

    }

    @Override
    public double kROhms() {
        return 0.12;
    }

    @Override
    public double kTNm_amp() {
        return 0.009;
    }

    public static Feedforward100 ff(LoggerFactory log) {
        // TODO: friction?
        return new Feedforward100(log, 11000, 0, 0,
                new Friction(log, 0, 0.07, 0.055, 0));
    }
}
