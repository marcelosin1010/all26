package org.team100.frc2026.robot;

import org.team100.lib.config.AutonChooser;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Populates the Auton chooser with all available autons.
 * 
 * It's a good idea to instantiate them all here, even if you're not using them
 * all, even if they're just in development, so they don't rot.
 */
public class AllAutons {
    private final AutonChooser m_autonChooser;

    public AllAutons(Machinery machinery) {
        m_autonChooser = new AutonChooser();

    }

    public Command get() {
        return m_autonChooser.get().command();
    }

    public void close() {
        m_autonChooser.close();
    }

}
