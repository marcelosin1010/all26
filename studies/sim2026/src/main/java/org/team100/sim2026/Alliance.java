package org.team100.sim2026;

import org.team100.sim2026.robots.Defender;
import org.team100.sim2026.robots.ExampleRobot;
import org.team100.sim2026.robots.Robot;
import org.team100.sim2026.robots.Scorer;

public class Alliance {
    final Robot robot1, robot2, robot3;

    public Alliance(Robot r1, Robot r2, Robot r3) {
        robot1 = r1;
        robot2 = r2;
        robot3 = r3;
    }

    public static Alliance balanced(AllianceColor color, SimRun sim) {
        return new Alliance(
                new Scorer(color, "1", 8, sim),
                new ExampleRobot(color, "2", 8, sim),
                new Defender(color, "3", 8, sim));
    }

    public static Alliance uniform(AllianceColor color, SimRun sim) {
        return new Alliance(
                new ExampleRobot(color, "1", 8, sim),
                new ExampleRobot(color, "2", 8, sim),
                new ExampleRobot(color, "3", 8, sim));
    }
}
