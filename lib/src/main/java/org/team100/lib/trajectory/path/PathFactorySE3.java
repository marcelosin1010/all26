package org.team100.lib.trajectory.path;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.WaypointSE3;
import org.team100.lib.trajectory.path.spline.SplineSE3;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist3d;

public class PathFactorySE3 {
    private static final boolean DEBUG = false;
    private static final double SPLINE_SAMPLE_TOLERANCE_M = 0.02;
    private static final double SPLINE_SAMPLE_TOLERANCE_RAD = 0.2;
    private static final double TRAJECTORY_STEP_M = 0.1;
    private final double maxNorm;
    private final double maxDx;
    private final double maxDTheta;

    public PathFactorySE3() {
        this(TRAJECTORY_STEP_M,
                SPLINE_SAMPLE_TOLERANCE_M,
                SPLINE_SAMPLE_TOLERANCE_RAD);
    }

    public PathFactorySE3(
            double maxNorm,
            double maxDx,
            double maxDTheta) {
        this.maxNorm = maxNorm;
        this.maxDx = maxDx;
        this.maxDTheta = maxDTheta;
    }

    public PathSE3 fromWaypoints(List<WaypointSE3> waypoints) {
        List<SplineSE3> splines = splinesFromWaypoints(waypoints);
        return fromSplines(splines);
    }

    private List<SplineSE3> splinesFromWaypoints(List<WaypointSE3> waypoints) {
        List<SplineSE3> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new SplineSE3(waypoints.get(i - 1), waypoints.get(i)));
        }
        return splines;
    }

    List<PathPointSE3> samplesFromSpline(SplineSE3 spline) {
        List<PathPointSE3> result = new ArrayList<>();
        result.add(spline.sample(0.0));
        getSegmentArc(spline, result, 0, 1);
        return result;
    }

    public PathSE3 fromSplines(List<? extends SplineSE3> splines) {
        return new PathSE3(samplesFromSplines(splines));
    }

    public List<PathPointSE3> samplesFromSplines(List<? extends SplineSE3> splines) {
        List<PathPointSE3> result = new ArrayList<>();
        if (splines.isEmpty())
            return result;
        result.add(splines.get(0).sample(0.0));
        for (int i = 0; i < splines.size(); i++) {
            SplineSE3 s = splines.get(i);
            if (DEBUG)
                System.out.printf("SPLINE:\n%d\n%s\n", i, s);
            List<PathPointSE3> samples = samplesFromSpline(s);
            // the sample at the end of the previous spline is the same as the one for the
            // beginning of the next, so don't include it twice.
            samples.remove(0);
            result.addAll(samples);
        }
        return result;
    }

    private void getSegmentArc(
            SplineSE3 spline,
            List<PathPointSE3> rv,
            double s0,
            double s1) {
        Pose3d p0 = spline.sample(s0).waypoint().pose();
        double shalf = (s0 + s1) / 2;
        Pose3d phalf = spline.sample(shalf).waypoint().pose();
        Pose3d p1 = spline.sample(s1).waypoint().pose();

        // twist from p0 to p1
        Twist3d twist_full = p0.log(p1);
        // twist halfway from p0 to p1
        Twist3d twist_half = GeometryUtil.scale(twist_full, 0.5);
        // point halfway from p0 to p1
        Pose3d phalf_predicted = p0.exp(twist_half);
        // difference between twist and sample
        Transform3d error = phalf_predicted.minus(phalf);

        // also prohibit large changes in direction between points
        PathPointSE3 p20 = spline.sample(s0);
        PathPointSE3 p21 = spline.sample(s1);
        Twist3d p2t = p20.waypoint().course().minus(p21.waypoint().course());

        // note the extra conditions to avoid points too far apart.
        // checks both translational and l2 norms
        // also checks change in course
        if (Math.abs(error.getTranslation().getNorm()) > maxDx
                || Math.abs(error.getRotation().getAngle()) > maxDTheta
                || Metrics.translationalNorm(twist_full) > maxNorm
                || Metrics.l2Norm(twist_full) > maxNorm
                || Metrics.l2Norm(p2t) > maxNorm) {
            // add a point in between
            getSegmentArc(spline, rv, s0, shalf);
            getSegmentArc(spline, rv, shalf, s1);
        } else {
            // midpoint is close enough, so add the endpoint
            rv.add(spline.sample(s1));
        }
    }

}
