package org.team100.frc2026;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberExtension extends SubsystemBase {
     private final BareMotor m_motor;

     private final double m_maxExtension=20;

    public ClimberExtension(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        m_motor = new SimulatedBareMotor(log, 600);
    }
    public Command setPosition() {
        return run(this::setOutPosition);
    }
    public Command setHomePosition() {
        return run(this::setInPosition);
    }
    private void setOutPosition() {
        m_motor.setUnwrappedPosition(m_maxExtension,0,0,0);
    
    }
    public void setInPosition() {
        m_motor.setUnwrappedPosition(0,0,0,0);
    }
}
