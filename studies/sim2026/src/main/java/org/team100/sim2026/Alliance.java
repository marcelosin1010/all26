package org.team100.sim2026;

import org.team100.sim2026.robots.Defender;
import org.team100.sim2026.robots.Ferry;
import org.team100.sim2026.robots.Lob;
import org.team100.sim2026.robots.Robot;
import org.team100.sim2026.robots.Scorer;

public class Alliance {
    final String name;
    final Robot robot1, robot2, robot3;

    public Alliance(String name, Robot r1, Robot r2, Robot r3) {
        this.name = name;
        robot1 = r1;
        robot2 = r2;
        robot3 = r3;
    }

    public static Alliance balanced(
            String name, AllianceColor color, SimRun sim) {
        return new Alliance(
                name,
                new Scorer(color, "1", 50, 25, 10, 8, sim),
                new Ferry(color, "2", 50, 25, 10, 8, sim),
                new Defender(color, "3", 50, 25, 10, 8, sim));
    }

    public static Alliance ferryOnly(
            String name, AllianceColor color, SimRun sim) {
        return new Alliance(
                name,
                new Ferry(color, "1", 50, 25, 10, 8, sim),
                new Ferry(color, "2", 50, 25, 10, 8, sim),
                new Ferry(color, "3", 50, 25, 10, 8, sim));
    }

    public static Alliance lobOnly(
            String name, AllianceColor color, SimRun sim) {
        return new Alliance(
                name,
                new Lob(color, "1", 50, 25, 10, 8, sim),
                new Lob(color, "2", 50, 25, 10, 8, sim),
                new Lob(color, "3", 50, 25, 10, 8, sim));
    }

    public static Alliance ferryOnly(
            String name, int capacity, int intakeRate, int shootRate, AllianceColor color, SimRun sim) {
        return new Alliance(
                name,
                new Ferry(color, "1", capacity, intakeRate, shootRate, 8, sim),
                new Ferry(color, "2", capacity, intakeRate, shootRate, 8, sim),
                new Ferry(color, "3", capacity, intakeRate, shootRate, 8, sim));
    }
}
