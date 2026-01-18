package org.team100.sim2026.robots;

import org.team100.sim2026.AllianceColor;
import org.team100.sim2026.SimRun;
import org.team100.sim2026.actions.Climb;

/**
 * Stays in our zone the whole time, scoring while active.
 */
public class Scorer extends Robot {
    private static final int CLIMB_TIME = 5;
    private static final int CLIMB_BUFFER = 10;

    public Scorer(
            AllianceColor alliance,
            String name,
            int initialCount,
            SimRun sim) {
        super(
                alliance,
                name,
                initialCount,
                sim);
    }

    @Override
    void action() {
        if (sim.time() < 20) {
            // auton
            intakeAndScore();
        } else if (sim.time() >= SimRun.MATCH_LENGTH_SEC - (CLIMB_BUFFER + CLIMB_TIME)) {
            if (action.getClass() == Climb.class)
                return;
            // time to climb, and not already climbing.
            action = new Climb(myTower, CLIMB_TIME);
        } else {
            if (active) {
                intakeAndScore();
            } else {
                intakeOnly();
            }
        }
    }

}
