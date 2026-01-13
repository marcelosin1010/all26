package org.team100.lib.trajectory.spline;

import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.path.PathSE2Entry;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;

/** Pose and curvature vector with arbitrary parameter */
public interface ISplineSE2 {
    Pose2d pose(double s);

    Vector<N2> K(double s);

    PathSE2Entry entry(double d);

    WaypointSE2 waypoint(double s0);
}
