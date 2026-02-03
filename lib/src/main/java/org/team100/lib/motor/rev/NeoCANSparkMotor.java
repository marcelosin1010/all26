package org.team100.lib.motor.rev;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.RevPIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.util.CanId;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/**
 * Neo motor.
 * 
 * @see https://www.revrobotics.com/rev-21-1650/
 */
public class NeoCANSparkMotor extends CANSparkMotor {
    public NeoCANSparkMotor(
            LoggerFactory parent,
            CanId canId,
            NeutralMode neutral,
            MotorPhase motorPhase,
            int statorCurrentLimit,
            Feedforward100 ff,
            RevPIDConstants pid) {
        super(parent, new SparkMax(canId.id, MotorType.kBrushless),
                neutral, motorPhase, statorCurrentLimit, ff, pid);
    }

    /** Real or simulated depending on identity */
    public static BareMotor get(
            LoggerFactory log, CanId can, MotorPhase phase, int statorLimit,
            Feedforward100 ff, RevPIDConstants pid) {
        return switch (Identity.instance) {
            case BLANK -> new SimulatedBareMotor(log, 600);
            default -> new NeoCANSparkMotor(
                    log, can, NeutralMode.BRAKE, phase, statorLimit, ff, pid);
        };
    }

    @Override
    public double kROhms() {
        return 0.114;
    }

    @Override
    public double kTNm_amp() {
        return 0.028;
    }

    public static Feedforward100 ff(LoggerFactory log) {
        // TODO: friction
        // TODO: verify kA
        return new Feedforward100(log, 5676, 0.01, 0.01,
                new Friction(log, 0.5, 0.5, 0.0, 0.5));
    }
}
