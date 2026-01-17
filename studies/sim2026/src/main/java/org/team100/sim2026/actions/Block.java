package org.team100.sim2026.actions;

import org.team100.sim2026.robots.Robot;

public class Block implements Action {

    private final Robot target;

    public Block(Robot target) {
        this.target = target;
    }

    public Robot target() {
        return target;
    }

    @Override
    public String toString() {
        return "block";
    }

}
    