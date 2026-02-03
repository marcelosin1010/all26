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

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Neo Vortex motor.
 * 
 * @see https://www.revrobotics.com/rev-21-1652/
 */
public class NeoVortexCANSparkMotor extends CANSparkMotor {
    public NeoVortexCANSparkMotor(
            LoggerFactory parent,
            CanId canId,
            NeutralMode neutral,
            MotorPhase motorPhase,
            int statorCurrentLimit,
            Feedforward100 ff,
            PIDConstants pid) {
        super(parent, new SparkFlex(canId.id, MotorType.kBrushless),
                neutral, motorPhase, statorCurrentLimit, ff, pid);
    }

    /**
     * Real or simulated depending on identity.
     * 
     * PID units for outboard velocity control are duty cycle per RPM, so if you
     * want to control to a few hundred RPM, P should be something like 0.0002.
     */
    public static BareMotor get(
            LoggerFactory log, CanId can, MotorPhase phase, int statorLimit,
            Feedforward100 ff, PIDConstants pid) {
        return switch (Identity.instance) {
            case BLANK ->
                new SimulatedBareMotor(log, 600);
            default -> new NeoVortexCANSparkMotor(
                    log, can, NeutralMode.BRAKE, phase, statorLimit, ff, pid);
        };

    }

    @Override
    public double kROhms() {
        return 0.057;
    }

    @Override
    public double kTNm_amp() {
        return 0.017;
    }

    public static Feedforward100 ff(LoggerFactory log) {
        // TODO: friction
        return new Feedforward100(log, 6784, 0.000, 0.000,
                new Friction(log, 0.100, 0.065, 0.0, 0.5));
    }
}
