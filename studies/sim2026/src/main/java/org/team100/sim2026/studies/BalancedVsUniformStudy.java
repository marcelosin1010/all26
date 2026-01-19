package org.team100.sim2026.studies;

import org.team100.sim2026.SimRun;
import org.team100.sim2026.scenarios.BalancedVsBalanced;
import org.team100.sim2026.scenarios.BalancedVsFerry;
import org.team100.sim2026.scenarios.FerryVsFerry;
import org.team100.sim2026.scenarios.FerryVsLob;
import org.team100.sim2026.scenarios.Scenario;

public class BalancedVsUniformStudy implements Runnable {
    @Override
    public void run() {
        System.out.println("Balanced vs Ferry");
        Scenario scenario = new BalancedVsFerry();
        SimRun sim = new SimRun(scenario, false);
        sim.run();

        System.out.println("Balanced vs Balanced");
        scenario = new BalancedVsBalanced();
        sim = new SimRun(scenario, false);
        sim.run();

        System.out.println("Ferry vs Ferry");
        scenario = new FerryVsFerry();
        sim = new SimRun(scenario, false);
        sim.run();

        System.out.println("Ferry vs Lob");
        scenario = new FerryVsLob();
        sim = new SimRun(scenario, false);
        sim.run();
    }
}
