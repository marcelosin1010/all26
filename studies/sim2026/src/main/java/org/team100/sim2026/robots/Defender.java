package org.team100.sim2026.robots;

import org.team100.sim2026.AllianceColor;
import org.team100.sim2026.SimRun;

/**
 * Active: Lob from the neutral zone.
 * Inactive: Defend in the opposite zone.
 */
public class Defender extends Robot {

    public Defender(AllianceColor alliance, String name, int capacity,
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
        lobOnly();
    }

    @Override
    void inactive() {
        defendInOppositeZone();
    }

}
