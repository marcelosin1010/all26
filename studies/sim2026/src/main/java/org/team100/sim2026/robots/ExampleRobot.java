package org.team100.sim2026.robots;

import org.team100.sim2026.AllianceColor;
import org.team100.sim2026.SimRun;
import org.team100.sim2026.actions.Climb;

/** Ferry during our active time, otherwise lob. */
public class ExampleRobot extends Robot {
    private static final int CLIMB_TIME = 5;
    private static final int CLIMB_BUFFER = 10;

    public ExampleRobot(
            AllianceColor alliance,
            String name,
            int initialCount,
            SimRun sim) {
        super(alliance,
                name,
                initialCount,
                sim);
    }

    void action() {
        if (sim.time() < 20) {
            // auton
            ferry();
        } else if (location == neutralZone
                && sim.time() >= SimRun.MATCH_LENGTH_SEC - (CLIMB_BUFFER + CLIMB_TIME + TRAVEL_TIME)) {
            // time to drive to our zone.
            moveTo(myZone);
        } else if (location == myZone
                && sim.time() >= SimRun.MATCH_LENGTH_SEC - (CLIMB_BUFFER + CLIMB_TIME)) {
            if (action.getClass() == Climb.class)
                return;
            // time to climb.
            action = new Climb(myTower, CLIMB_TIME);
        } else {
            // teleop
            if (active) {
                ferry();
            } else {
                lob();
            }
        }
    }

}
