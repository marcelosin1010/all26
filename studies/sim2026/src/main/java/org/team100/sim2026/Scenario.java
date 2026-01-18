package org.team100.sim2026;

import org.team100.sim2026.robots.Defender;
import org.team100.sim2026.robots.ExampleRobot;
import org.team100.sim2026.robots.Scorer;

/** Factory for alliances for a single simulation run. */
public class Scenario {
    public Alliance red(SimRun sim) {
        // multi-role
        return new Alliance(
                new Scorer(AllianceColor.RED, "r1", 8, sim),
                new ExampleRobot(AllianceColor.RED, "r2", 8, sim),
                new Defender(AllianceColor.RED, "r3", 8, sim));
    }

    public Alliance blue(SimRun sim) {
        // all-around
        return new Alliance(
                new ExampleRobot(AllianceColor.BLUE, "b1", 8, sim),
                new ExampleRobot(AllianceColor.BLUE, "b2", 8, sim),
                new ExampleRobot(AllianceColor.BLUE, "b3", 8, sim));
    }

    Alliance blueB(SimRun sim) {
        // multi-role
        return new Alliance(
                new Scorer(AllianceColor.BLUE, "b1", 8, sim),
                new ExampleRobot(AllianceColor.BLUE, "b2", 8, sim),
                new Defender(AllianceColor.BLUE, "b3", 8, sim));
    }
}
