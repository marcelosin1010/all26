package org.team100.sim2026.actions;

public class IntakeAndScore implements Action {
    private int taken;
    private int shot;

    public IntakeAndScore(int taken, int shot) {
        this.taken = taken;
        this.shot = shot;
    }

    @Override
    public String toString() {
        return String.format("in %2d score %2d", taken, shot);
    }

}
