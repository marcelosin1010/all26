package org.team100.sim2026.robots;

import org.team100.sim2026.AllianceColor;
import org.team100.sim2026.SimRun;

/**
 * Inactive: Lob from the neutral zone.
 * Active: Ferry and score.
 */
public class Ferry extends Robot {

    public Ferry(AllianceColor alliance, String name, int capacity,
            int intakeRate, int shootRate, int initialCount, SimRun sim) {
        super(alliance, name, capacity,
                intakeRate, shootRate, initialCount, sim);
    }

    @Override
    void auton() {
        ferryAndScore();
    }

    @Override
    void active() {
        ferryAndScore();
    }

    @Override
    void inactive() {
        lob();
    }

}
