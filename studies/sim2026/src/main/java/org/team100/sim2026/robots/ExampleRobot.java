package org.team100.sim2026.robots;

import java.util.function.IntSupplier;

import org.team100.sim2026.Hub;
import org.team100.sim2026.Sim;
import org.team100.sim2026.Tower;
import org.team100.sim2026.Zone;
import org.team100.sim2026.actions.Climb;

/** Ferry during our active time, otherwise lob. */
public class ExampleRobot extends Robot {
    private static final int CLIMB_TIME = 5;
    private static final int CLIMB_BUFFER = 10;

    public ExampleRobot(
            String name,
            Zone myZone,
            Zone neutralZone,
            Zone otherZone,
            Hub myHub,
            Tower myTower,
            int initialCount,
            IntSupplier matchTimer) {
        super(
                name,
                myZone,
                neutralZone,
                otherZone,
                myHub,
                myTower,
                initialCount,
                matchTimer);
    }

    void action() {
        if (matchTimer.getAsInt() < 20) {
            // auton
            ferry();
        } else if (location == neutralZone
                && matchTimer.getAsInt() >= Sim.MATCH_LENGTH_SEC - (CLIMB_BUFFER + CLIMB_TIME + TRAVEL_TIME)) {
            // System.out.println("go climb");
            // time to go climb, and not already moving.
            moveTo(myZone);
        } else if (location == myZone
                && matchTimer.getAsInt() >= Sim.MATCH_LENGTH_SEC - (CLIMB_BUFFER + CLIMB_TIME)) {
            if (action.getClass() == Climb.class)
                return;
            // time to climb, and not already climbing.
            // System.out.println("climb");
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
