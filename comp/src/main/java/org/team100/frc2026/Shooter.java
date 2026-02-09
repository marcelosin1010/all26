package org.team100.frc2026;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final BareMotor m_motor;

    public Shooter(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        m_motor = new SimulatedBareMotor(log, 600);
    }

    @Override
    public void periodic() {
        m_motor.periodic();
    }

    public Command shoot() {
        return run(this::fullSpeed);
    }

    public Command stop() {
        return run(this::stopMotor).withName("stop");
    }

    public void stopMotor() {
        m_motor.setDutyCycle(0);
    }

    private void fullSpeed() {
        m_motor.setDutyCycle(1);
    }

}
