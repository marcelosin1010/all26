package org.team100.lib.motor.rev;

import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
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
            NeutralMode100 neutral,
            MotorPhase motorPhase,
            int statorCurrentLimit,
            SimpleDynamics ff,
            Friction friction,
            PIDConstants pid) {
        super(parent, new SparkMax(canId.id, MotorType.kBrushless),
                neutral, motorPhase, statorCurrentLimit, ff, friction, pid);
    }

    /** Real or simulated depending on identity */
    public static BareMotor get(
            LoggerFactory log, CanId can, MotorPhase phase, int statorLimit,
            SimpleDynamics ff, Friction friction, PIDConstants pid) {
        return switch (Identity.instance) {
            case BLANK -> new SimulatedBareMotor(log, 600);
            default -> new NeoCANSparkMotor(
                    log, can, NeutralMode100.BRAKE, phase, statorLimit, ff, friction, pid);
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

    @Override
    public double kFreeSpeedRPM() {
        return 5676;
    }

    public static SimpleDynamics ff(LoggerFactory log) {
        // TODO: verify kA
        return new SimpleDynamics(log, 0.01, 0.01);
    }

    public static Friction friction(LoggerFactory log) {
        // TODO: friction
        return new Friction(log, 0.5, 0.5, 0.0, 0.5);
    }
}
