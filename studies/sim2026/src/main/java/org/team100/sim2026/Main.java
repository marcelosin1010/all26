package org.team100.sim2026;

import org.team100.sim2026.studies.BalancedVsUniformStudy;
import org.team100.sim2026.studies.CapacityStudy;
import org.team100.sim2026.studies.IntakeStudy;
import org.team100.sim2026.studies.ShootingStudy;

public class Main {
    public static void main(String... args) {
        System.out.println("Simulation starting ...");

        System.out.println("\nROLES");
        Runnable study = new BalancedVsUniformStudy();
        study.run();

        System.out.println("\nCAPACITY");
        study = new CapacityStudy();
        study.run();

        System.out.println("\nINTAKE");
        study = new IntakeStudy();
        study.run();

        System.out.println("\nSHOOTING");
        study = new ShootingStudy();
        study.run();

        System.out.println("\nSimulation done.");
    }

}
