package org.team100.lib.trajectory.timing;

import org.team100.lib.trajectory.path.PathPointSE2;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A constant velocity limit within a rectangle; no limit outside.
 */
public class VelocityLimitRegionConstraint implements TimingConstraint {
    private final Translation2d m_min;
    private final Translation2d m_max;
    private final double m_limit;

    public VelocityLimitRegionConstraint(
            Translation2d min_corner,
            Translation2d max_corner,
            double velocity_limit) {
        if (velocity_limit < 0)
            throw new IllegalArgumentException();
        m_min = min_corner;
        m_max = max_corner;
        m_limit = velocity_limit;
    }

    @Override
    public double maxV(PathPointSE2 state) {
        final Translation2d translation = state.waypoint().pose().getTranslation();
        if (translation.getX() <= m_max.getX() && translation.getX() >= m_min.getX() &&
                translation.getY() <= m_max.getY() && translation.getY() >= m_min.getY()) {
            return m_limit;
        }
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public double maxAccel(PathPointSE2 state, double velocity) {
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public double maxDecel(PathPointSE2 state, double velocity) {
        return Double.NEGATIVE_INFINITY;
    }

}