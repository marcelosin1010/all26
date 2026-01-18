package org.team100.sim2026.robots;

import org.team100.sim2026.Alliance;
import org.team100.sim2026.Sim;
import org.team100.sim2026.actions.Climb;

/**
 * Lobs in the neutral zone while active.
 * Defends in the opposite zone while inactive.
 */
public class Defender extends Robot {
    private static final int CLIMB_TIME = 5;
    private static final int CLIMB_BUFFER = 10;

    public Defender(
            Alliance alliance,
            String name,
            int initialCount,
            Sim sim) {
        super(
                alliance,
                name,
                initialCount,
                sim);
    }

    void action() {
        if (sim.time() < 20) {
            // auton
            ferry();
        } else if (location == otherZone
                && sim.time() >= Sim.MATCH_LENGTH_SEC - (CLIMB_BUFFER + CLIMB_TIME + 2 * TRAVEL_TIME)) {
            // time to drive to the neutral zone
            moveTo(neutralZone);
        } else if (location == neutralZone
                && sim.time() >= Sim.MATCH_LENGTH_SEC - (CLIMB_BUFFER + CLIMB_TIME + TRAVEL_TIME)) {
            // time to drive to our zone.
            moveTo(myZone);
        } else if (location == myZone
                && sim.time() >= Sim.MATCH_LENGTH_SEC - (CLIMB_BUFFER + CLIMB_TIME)) {
            if (action.getClass() == Climb.class)
                return;
            // time to climb.
            action = new Climb(myTower, CLIMB_TIME);
        } else {
            // teleop
            if (active) {
                lobOnly();
            } else {
                defendInOppositeZone();
            }
        }

    }

}
