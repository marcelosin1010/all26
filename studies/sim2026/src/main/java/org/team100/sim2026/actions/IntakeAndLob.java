package org.team100.sim2026.actions;

public class IntakeAndLob implements Action {
    private int taken;
    private int lobbed;

    public IntakeAndLob(int taken, int shot) {
        this.taken = taken;
        this.lobbed = shot;
    }

    @Override
    public String toString() {
        return String.format("in %2d lob %2d", taken, lobbed);
    }
}
