package org.team100.sim2026;

/** Factory for alliances for a single simulation run. */
public class Scenario {
    public Alliance red(SimRun sim) {
        return Alliance.balanced(AllianceColor.RED, sim);
    }

    public Alliance blue(SimRun sim) {
        return Alliance.uniform(AllianceColor.BLUE, sim);
    }
}
