package org.team100.lib.trajectory.path;

import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.path.spline.SplineSE2;
import org.team100.lib.util.Math100;

/**
 * Represents a point on a path in SE(2) (plane with rotation).
 * 
 * Includes a WaypointSE2, heading rate, and curvature.
 */
public class PathPointSE2 {
    private static final boolean DEBUG = false;
    /** Pose and course. */
    private final WaypointSE2 m_waypoint;
    /** The source of this point (for resampling) */
    private final SplineSE2 m_spline;
    /** The parameter value of this point (for resampling) */
    private final double m_s;
    /** Change in heading per meter of motion, rad/m. */
    private final double m_headingRateRad_M;
    /**
     * Change in course per change in distance, rad/m.
     * This is a signed quantity indicating the direction from the tangent vector
     * (course), it's not just the magnitude of curvature.
     */
    private final double m_curvatureRad_M;

    /**
     * @param waypoint         location and heading and direction of travel
     * @param headingRateRad_M change in heading, per meter traveled
     * @param curvatureRad_M   change in course per meter traveled.
     */
    public PathPointSE2(
            WaypointSE2 waypoint,
            SplineSE2 spline,
            double s,
            double headingRateRad_M,
            double curvatureRad_M) {
        m_waypoint = waypoint;
        m_spline = spline;
        m_s = s;
        m_headingRateRad_M = headingRateRad_M;
        m_curvatureRad_M = curvatureRad_M;
    }

    public PathPointSE2(
            WaypointSE2 waypoint,
            double headingRateRad_M,
            double curvatureRad_M) {
        m_waypoint = waypoint;
        m_spline = null;
        m_s = 0;
        m_headingRateRad_M = headingRateRad_M;
        m_curvatureRad_M = curvatureRad_M;
    }

    public WaypointSE2 waypoint() {
        return m_waypoint;
    }

    public double getS() {
        return m_s;
    }

    /**
     * Heading rate is radians per meter.
     * 
     * If you want radians per second, multiply by velocity (meters per second).
     */
    public double getHeadingRateRad_M() {
        return m_headingRateRad_M;
    }

    /** Radians per meter, which is the reciprocal of the radius. */
    public double getCurvatureRad_M() {
        return m_curvatureRad_M;
    }

    /**
     * Interpolates by resampling the underlying spline.
     * 
     * The parameter is assumed to vary linearly between points, so the interpolant
     * here is just a fraction of that linear variation.
     */
    public PathPointSE2 interpolate(PathPointSE2 other, double x) {
        if (DEBUG)
            System.out.printf("this s %f other s %f\n",
                    m_s, other.m_s);
        SplineSE2 spline = null;
        double s = 0;
        if (m_spline == other.m_spline) {
            // ok to interpolate using this spline
            if (DEBUG)
                System.out.println("same spline");
            spline = m_spline;
            s = Math100.interpolate(m_s, other.m_s, x);
        } else {
            // which one to use?
            // one of the endpoints should be the spline endpoint
            // which is always the zero (not the 1)
            if (other.m_s < 1e-6) {
                // other one is the start, so use this one
                if (DEBUG)
                    System.out.println("use this spline");
                spline = m_spline;
                s = Math100.interpolate(m_s, 1, x);
            } else {
                if (DEBUG)
                    System.out.println("use the other spline");
                spline = other.m_spline;
                s = Math100.interpolate(0, other.m_s, x);
            }
        }
        if (DEBUG)
            System.out.printf("s0 %f s1 %f x %f s %f\n",
                    m_s, other.m_s, x, s);

        if (spline != null)
            return spline.sample(s);

        new Throwable().printStackTrace();
        throw new IllegalStateException();
    }

    /**
     * R2 (xy) planar distance only (IGNORES ROTATION) so that planar
     * velocity and curvature works correctly. Not the twist arclength.
     * Not the double-geodesic L2 thing. Just XY translation hypot.
     * 
     * Always non-negative.
     */
    public double distanceCartesian(PathPointSE2 other) {
        return Metrics.translationalDistance(m_waypoint.pose(), other.m_waypoint.pose());
    }

    public boolean equals(Object other) {
        if (!(other instanceof PathPointSE2)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }

        PathPointSE2 p2dwc = (PathPointSE2) other;
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
        if (!Math100.epsilonEquals(m_curvatureRad_M, p2dwc.m_curvatureRad_M)) {
            if (DEBUG)
                System.out.println("wrong curvature");
            return false;
        }
        return true;
    }

    public String toString() {
        return String.format(
                "x %5.3f, y %5.3f, theta %5.3f, course %s, dtheta %5.3f, curvature %5.3f",
                m_waypoint.pose().getTranslation().getX(),
                m_waypoint.pose().getTranslation().getY(),
                m_waypoint.pose().getRotation().getRadians(),
                m_waypoint.course(),
                m_headingRateRad_M,
                m_curvatureRad_M);
    }

}