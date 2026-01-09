package org.team100.lib.trajectory.spline;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;

/**
 * A path defined as an offset from a spline.
 * 
 * The length of the offset is fixed; the angle is the reciprocal of the
 * heading.
 * 
 * The use-case is to define the spline as the toolpoint, and the offset as
 * whatever is carrying the toolpoint around, e.g. the drivetrain.
 * 
 * That way, we can do three things correctly:
 * 
 * * "drawings" of the toolpoint spline, using the real work site
 * * optimization of the trajectory schedule of the drivetrain
 * * runtime feedback using the toolpoint, not the drivetrain.
 * 
 */
public class OffsetSE2 {

    private final SplineSE2 m_toolpoint;
    private final double m_length;

    public OffsetSE2(SplineSE2 toolpoint, double length) {
        m_toolpoint = toolpoint;
        m_length = length;
    }

    public Pose2d pose(double s) {
        Vector<N2> p = SplineUtil.offsetR(m_toolpoint, m_length, s);
        return new Pose2d(p.get(0), p.get(1), m_toolpoint.pose(s).getRotation());
    }

}
