package org.team100.lib.subsystems.shooter;

import org.team100.lib.config.RevPIDConstants;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.rev.Neo550CANSparkMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {

    private final BareMotor m_pivot;
    private final DoubleLogger m_log_angle;

    public PivotSubsystem(LoggerFactory parent, int currentLimit) {
        LoggerFactory logger = parent.type(this);
        m_log_angle = logger.doubleLogger(Level.TRACE, "Angle (rad)");
        m_pivot = Neo550CANSparkMotor.get(
                logger,
                new CanId(5),
                MotorPhase.FORWARD,
                currentLimit,
                Neo550CANSparkMotor.ff(logger),
                RevPIDConstants.zero(logger));
    }

    public void dutyCycle(double set) {
        m_pivot.setDutyCycle(set);
    }

    public double getAngleRad() {
        return m_pivot.getUnwrappedPositionRad();
    }

    public void setEncoderPosition(double positionRad) {
        m_pivot.setUnwrappedEncoderPositionRad(positionRad);
    }

    public void setTorqueLimit(double value) {
        m_pivot.setTorqueLimit(value);
    }

    public void stop() {
        m_pivot.stop();
    }

    @Override
    public void periodic() {
        m_pivot.periodic();
        m_log_angle.log(this::getAngleRad);
    }
}
