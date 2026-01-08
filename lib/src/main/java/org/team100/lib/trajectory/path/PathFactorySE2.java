package org.team100.lib.trajectory.path;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.trajectory.path.spline.SplineSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;

public class PathFactorySE2 {
    private static final boolean DEBUG = false;
    /*
     * Maximum distance of the secant lines to the continuous spline. The resulting
     * path will have little scallops if it involves rotation. In SE(2), a constant
     * "twist" segment with rotation is a curve. If the scallops are too big, make
     * this number smaller. If the trajectories are too slow to generate, make this
     * number bigger.
     */
    private static final double SPLINE_SAMPLE_TOLERANCE_M = 0.02;
    /**
     * Maximum theta error.
     */
    private static final double SPLINE_SAMPLE_TOLERANCE_RAD = 0.2;
    /**
     * Size of steps along the path. Make this number smaller for tight curves to
     * look better. Make it bigger to make trajectories (a little) faster to
     * generate.
     */
    private static final double TRAJECTORY_STEP_M = 0.1;

    private final double m_maxNorm;
    private final double m_maxDx;
    private final double m_maxDy;
    private final double m_maxDTheta;

    public PathFactorySE2() {
        this(TRAJECTORY_STEP_M,
                SPLINE_SAMPLE_TOLERANCE_M,
                SPLINE_SAMPLE_TOLERANCE_M,
                SPLINE_SAMPLE_TOLERANCE_RAD);
    }

    public PathFactorySE2(
            double maxNorm,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        m_maxNorm = maxNorm;
        m_maxDx = maxDx;
        m_maxDy = maxDy;
        m_maxDTheta = maxDTheta;
    }

    /**
     * Converts a list of SplineSE2 into a PathSE2.
     * 
     * The points are chosen so that the secant line between the points is within
     * the specified tolerance (dx, dy, dtheta) of the actual spline.
     * 
     * The trajectory scheduler consumes these points, interpolating between them
     * with straight lines.
     * 
     * TODO: explore performance of this part, it could be faster.
     */
    public PathSE2 get(List<? extends SplineSE2> splines) {
        List<PathEntrySE2> result = new ArrayList<>();
        if (splines.isEmpty())
            return new PathSE2(result);
        result.add(splines.get(0).entry(0.0));
        for (int i = 0; i < splines.size(); i++) {
            SplineSE2 s = splines.get(i);
            if (DEBUG)
                System.out.printf("SPLINE:\n%d\n%s\n", i, s);
            addEndpointOrBisect(s, result, 0, 1);
        }
        return new PathSE2(result);
    }

    /////////////////////////////////////////////////////////////////////////////////////
    ///
    ///

    /**
     * Recursive bisection to find a series of secant lines close to the real curve,
     * and with the points closer than maxNorm to each other, measured in L2 norm
     * (i.e. x, y, heading), and also course.
     * 
     * Note if the path is s-shaped, then bisection can find the middle :-)
     */
    void addEndpointOrBisect(
            SplineSE2 spline,
            List<PathEntrySE2> rv,
            double s0,
            double s1) {
        Pose2d p0 = spline.sample(s0).waypoint().pose();
        double shalf = (s0 + s1) / 2;
        Pose2d phalf = spline.sample(shalf).waypoint().pose();
        Pose2d p1 = spline.sample(s1).waypoint().pose();

        // twist from p0 to p1
        Twist2d twist_full = p0.log(p1);
        // twist halfway from p0 to p1
        Twist2d twist_half = GeometryUtil.scale(twist_full, 0.5);
        // point halfway from p0 to p1
        Pose2d phalf_predicted = p0.exp(twist_half);
        // difference between twist and sample
        Transform2d error = phalf_predicted.minus(phalf);

        // also prohibit large changes in direction between points
        PathPointSE2 p20 = spline.sample(s0);
        PathPointSE2 p21 = spline.sample(s1);
        Twist2d p2t = p20.waypoint().course().minus(p21.waypoint().course());

        // note the extra conditions to avoid points too far apart.
        // checks both translational and l2 norms
        // also checks change in course
        if (Math.abs(error.getTranslation().getX()) > m_maxDx
                || Math.abs(error.getTranslation().getY()) > m_maxDy
                || Math.abs(error.getRotation().getRadians()) > m_maxDTheta
                || Metrics.translationalNorm(twist_full) > m_maxNorm
                || Metrics.l2Norm(twist_full) > m_maxNorm
                || Metrics.l2Norm(p2t) > m_maxNorm) {
            // add a point in between
            addEndpointOrBisect(spline, rv, s0, shalf);
            addEndpointOrBisect(spline, rv, shalf, s1);
        } else {
            // midpoint is close enough, so add the endpoint
            rv.add(spline.entry(s1));
        }
    }
}
