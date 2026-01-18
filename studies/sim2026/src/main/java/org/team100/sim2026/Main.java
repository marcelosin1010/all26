package org.team100.sim2026;

public class Main {
    public static void main(String... args) {
        System.out.println("simulation starting ...");
        Scenario scenario = new Scenario();
        SimRun sim = new SimRun(scenario);
        sim.run();
        System.out.println("simulation done.");
    }

}
