package org.team100.sim2026.scenarios;

import org.team100.sim2026.Alliance;
import org.team100.sim2026.AllianceColor;
import org.team100.sim2026.SimRun;

public class FerryVsFerryVariable implements Scenario {
    private final int redCapacity;
    private final int redIntakeRate;
    private final int redShootRate;
    private final int blueCapacity;
    private final int blueIntakeRate;
    private final int blueShootRate;

    public FerryVsFerryVariable(
            int redCapacity, int redIntakeRate, int redShootRate,
            int blueCapacity, int blueIntakeRate, int blueShootRate) {
        this.redCapacity = redCapacity;
        this.redIntakeRate = redIntakeRate;
        this.redShootRate = redShootRate;
        this.blueCapacity = blueCapacity;
        this.blueIntakeRate = blueIntakeRate;
        this.blueShootRate = blueShootRate;
    }

    @Override
    public Alliance red(SimRun sim) {
        return Alliance.ferryOnly(
                "ferry", redCapacity, redIntakeRate, redShootRate, AllianceColor.RED, sim);
    }

    @Override
    public Alliance blue(SimRun sim) {
        return Alliance.ferryOnly(
                "ferry", blueCapacity, blueIntakeRate, blueShootRate, AllianceColor.BLUE, sim);
    }
}
