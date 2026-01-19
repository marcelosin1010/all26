package org.team100.sim2026.studies;

import org.team100.sim2026.SimRun;
import org.team100.sim2026.scenarios.Scenario;
import org.team100.sim2026.scenarios.FerryVsFerryVariable;

public class IntakeStudy implements Runnable {
    private static final int CAPACITY = 50;
    // private static final int INTAKE_RATE = 25;
    private static final int SHOOT_RATE = 10;

    @Override
    public void run() {
        System.out.println("Red med blue low");
        Scenario scenario = new FerryVsFerryVariable(
                CAPACITY, 20, SHOOT_RATE, CAPACITY, 5, SHOOT_RATE);
        SimRun sim = new SimRun(scenario, false);
        sim.run();
        System.out.println("Red high Blue low");
        scenario = new FerryVsFerryVariable(
                CAPACITY, 40, SHOOT_RATE, CAPACITY, 5, SHOOT_RATE);
        sim = new SimRun(scenario, false);
        sim.run();
        System.out.println("Red high Blue med");
        scenario = new FerryVsFerryVariable(
                CAPACITY, 40, SHOOT_RATE, CAPACITY, 20, SHOOT_RATE);
        sim = new SimRun(scenario, false);
        sim.run();
    }

}
