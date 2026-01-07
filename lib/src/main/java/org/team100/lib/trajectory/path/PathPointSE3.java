package org.team100.lib.trajectory.path;

import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.WaypointSE3;
import org.team100.lib.trajectory.path.spline.SplineSE3;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

/**
 * Represents a point on a path in SE(3) (3d space with rotation).
 * 
 * Includes a WaypointSE3, heading rate, and curvature.
 */
public class PathPointSE3 {
    private static final boolean DEBUG = false;

    /** Pose and course. */
    private final WaypointSE3 m_waypoint;
    private final SplineSE3 m_spline;
    private final double m_s;
    /**
     * The curvature vector is the path-length-derivative of the unit tangent
     * vector. It's an R3 vector here but it's constrained to the plane
     * perpendicular to the tangent vector, T, i.e. the course.
     * 
     * It points in the direction of the center of curvature.
     * https://en.wikipedia.org/wiki/Center_of_curvature
     * 
     * Its magnitude is "κ", 1/radius of osculating circle, rad/m
     * https://en.wikipedia.org/wiki/Osculating_circle
     */
    private final Vector<N3> m_K;

    /**
     * The heading rate is the path-length derivative of the heading vector.
     */
    private final Vector<N3> m_H;

    /**
     * @param waypoint
     * @param K        curvature vector
     * @param H        path-length angular velocity of heading
     */
    public PathPointSE3(
            WaypointSE3 waypoint,
            SplineSE3 spline,
            double s,
            Vector<N3> K,
            Vector<N3> H) {
        m_waypoint = waypoint;
        m_spline = spline;
        Vector<N3> T = waypoint.course().translation();
        if (K.dot(T) > 1e-6)
            throw new IllegalArgumentException("K must be perpendicular to T");
        m_s = s;
        m_K = K;
        m_H = H;
    }

    public WaypointSE3 waypoint() {
        return m_waypoint;
    }

    public double getS() {
        return m_s;
    }

    public Vector<N3> curvature() {
        return m_K;
    }

    public Vector<N3> headingRate() {
        return m_H;
    }

    public double distanceCartesian(PathPointSE3 other) {
        return Metrics.translationalDistance(m_waypoint.pose(), other.m_waypoint.pose());
    }

    public PathPointSE3 interpolate(PathPointSE3 other, double x) {
        if (DEBUG)
            System.out.printf("this s %f other s %f\n",
                    m_s, other.m_s);
        SplineSE3 spline = null;
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
        // sample the spline again instead of interpolating.
        if (spline != null)
            return spline.sample(s);
        return null;
    }

}
