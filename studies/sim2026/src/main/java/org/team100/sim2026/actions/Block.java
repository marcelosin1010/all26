package org.team100.sim2026.actions;

import org.team100.sim2026.robots.Robot;

/** TODO: make Block also slow down travel */
public class Block implements Action {

    public final Robot target;

    public Block(Robot target) {
        this.target = target;
    }

    public Robot target() {
        return target;
    }

    @Override
    public String toString() {
        return String.format("block %s", target.name);
    }

}
