package org.team100.frc2026;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final BareMotor m_motor;
    // these are for the degrees that the motor moves
    private final double m_level1 = 90;
    private final double m_level3 = 180;

    public Climber(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        m_motor = new SimulatedBareMotor(log, 600);
    }

    public Command setClimb0() {
        return run(this::setL0);
    }

    public Command setClimb1() {
        return run(this::setL1);
    }

    public Command setClimb3() {
        return run(this::setL3);
    }

    private void setL0() {
        m_motor.setUnwrappedPosition(0, 0, 0, 0);

    }

    public void setL1() {
        m_motor.setUnwrappedPosition(m_level1, 0, 0, 0);
    }

    public void setL3() {
        m_motor.setUnwrappedPosition(m_level3, 0, 0, 0);
    }
}
