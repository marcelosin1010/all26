package org.team100.lib.trajectory.path;

import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.spline.SplineUtil;
import org.team100.lib.util.Math100;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

/**
 * Represents a point on a path in SE(2) (plane with rotation).
 * 
 * Includes a WaypointSE2, heading rate, and curvature.
 */
public class PathSE2Point {
    static final boolean DEBUG = false;
    /**
     * Pose and course.
     */
    private final WaypointSE2 m_waypoint;
    /**
     * Change in heading per meter of motion, rad/m.
     */
    private final double m_headingRateRad_M;
    /**
     * Curvature vector. Norm is \kappa, curvature in radians per meter, which is
     * the reciprocal of the radius.
     */
    private final Vector<N2> m_K;

    /**
     * @param waypoint         location and heading and direction of travel
     * @param headingRateRad_M change in heading, per meter traveled
     * @param curvatureRad_M   change in course per meter traveled.
     */
    public PathSE2Point(
            WaypointSE2 waypoint,
            double headingRateRad_M,
            Vector<N2> K) {
        m_waypoint = waypoint;
        m_headingRateRad_M = headingRateRad_M;
        m_K = K;
    }

    public WaypointSE2 waypoint() {
        return m_waypoint;
    }

    /**
     * Heading rate is radians per meter.
     * 
     * If you want radians per second, multiply by velocity (meters per second).
     */
    public double getHeadingRateRad_M() {
        return m_headingRateRad_M;
    }

    /**
     * Radians per meter, which is the reciprocal of the radius.
     * 
     * TODO: change the name of this method to kappa.
     */
    public double getCurvatureRad_M() {
        return SplineUtil.kappaSigned(m_waypoint.course().T(), m_K);
    }

    public Vector<N2> K() {
        return m_K;
    }

    /**
     * R2 (xy) planar distance only (IGNORES ROTATION) so that planar
     * velocity and curvature works correctly. Not the twist arclength.
     * Not the double-geodesic L2 thing. Just XY translation hypot.
     * 
     * Always non-negative.
     */
    public double distanceCartesian(PathSE2Point other) {
        return Metrics.translationalDistance(m_waypoint.pose(), other.m_waypoint.pose());
    }

    public boolean equals(Object other) {
        if (!(other instanceof PathSE2Point)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }

        PathSE2Point p2dwc = (PathSE2Point) other;
        if (!m_waypoint.equals(p2dwc.m_waypoint)) {
            if (DEBUG)
                System.out.println("wrong waypoint");
            return false;
        }
        if (!Math100.epsilonEquals(m_headingRateRad_M, p2dwc.m_headingRateRad_M)) {
            if (DEBUG)
                System.out.println("wrong heading rate");
            return false;
        }
        if (!Math100.epsilonEquals(m_K, p2dwc.m_K)) {
            if (DEBUG)
                System.out.println("wrong K");
            return false;
        }
        return true;
    }

    public String toString() {
        return String.format(
                "x %5.3f, y %5.3f, theta %5.3f, course %s, dtheta %5.3f, K %s",
                m_waypoint.pose().getTranslation().getX(),
                m_waypoint.pose().getTranslation().getY(),
                m_waypoint.pose().getRotation().getRadians(),
                m_waypoint.course(),
                m_headingRateRad_M,
                StrUtil.vecStr(m_K));
    }

}