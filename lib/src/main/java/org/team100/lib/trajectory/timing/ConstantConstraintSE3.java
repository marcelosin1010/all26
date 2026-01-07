package org.team100.lib.trajectory.timing;

import org.team100.lib.trajectory.path.PathPointSE3;

// TODO: a real version of this
public class ConstantConstraintSE3 implements TimingConstraintSE3 {

    @Override
    public double maxV(PathPointSE3 state) {
        return 1.0;
    }

    @Override
    public double maxAccel(PathPointSE3 state, double velocityM_S) {
        return 1.0;
    }

    @Override
    public double maxDecel(PathPointSE3 state, double velocityM_S) {
        return 1.0;
    }

}
