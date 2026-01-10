package org.team100.lib.trajectory.path;

import org.team100.lib.geometry.Metrics;
import org.team100.lib.trajectory.spline.SplineSE2;
import org.team100.lib.trajectory.spline.SplineSE3;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Twist2d;

public class PathUtil {
    private static final boolean DEBUG = false;

    /**
     * Interpolates by resampling the underlying spline.
     * 
     * The parameter is assumed to vary linearly between points, so the interpolant
     * here is just a fraction of that linear variation.
     */
    public static PathSE2Entry interpolate(PathSE2Entry a, PathSE2Entry b, double x) {
        if (DEBUG)
            System.out.printf("this s %f other s %f\n",
                    a.parameter().s(), b.parameter().s());
        SplineSE2 spline = null;
        double s = 0;
        if (a.parameter().spline() == b.parameter().spline()) {
            // ok to interpolate using this spline
            if (DEBUG)
                System.out.println("same spline");
            spline = a.parameter().spline();
            s = Math100.interpolate(a.parameter().s(), b.parameter().s(), x);
        } else {
            // which one to use?
            // one of the endpoints should be the spline endpoint
            // which is always the zero (not the 1)
            if (b.parameter().s() < 1e-6) {
                // other one is the start, so use this one
                if (DEBUG)
                    System.out.println("use this spline");
                spline = a.parameter().spline();
                s = Math100.interpolate(a.parameter().s(), 1, x);
            } else {
                if (DEBUG)
                    System.out.println("use the other spline");
                spline = b.parameter().spline();
                s = Math100.interpolate(0, b.parameter().s(), x);
            }
        }
        if (DEBUG)
            System.out.printf("s0 %f s1 %f x %f s %f\n",
                    a.parameter().s(), b.parameter().s(), x, s);

        if (spline != null)
            return spline.entry(s);

        new Throwable().printStackTrace();
        throw new IllegalStateException();
    }

    public static PathSE3Entry interpolate(PathSE3Entry a, PathSE3Entry b, double x) {
        if (DEBUG)
            System.out.printf("this s %f other s %f\n",
                    a.parameter().s(), b.parameter().s());
        SplineSE3 spline = null;
        double s = 0;
        if (a.parameter().spline() == b.parameter().spline()) {
            // ok to interpolate using this spline
            if (DEBUG)
                System.out.println("same spline");
            spline = a.parameter().spline();
            s = Math100.interpolate(a.parameter().s(), b.parameter().s(), x);
        } else {
            // which one to use?
            // one of the endpoints should be the spline endpoint
            // which is always the zero (not the 1)
            if (b.parameter().s() < 1e-6) {
                // other one is the start, so use this one
                if (DEBUG)
                    System.out.println("use this spline");
                spline = a.parameter().spline();
                s = Math100.interpolate(a.parameter().s(), 1, x);
            } else {
                if (DEBUG)
                    System.out.println("use the other spline");
                spline = b.parameter().spline();
                s = Math100.interpolate(0, b.parameter().s(), x);
            }
        }
        if (DEBUG)
            System.out.printf("s0 %f s1 %f x %f s %f\n",
                    a.parameter().s(), b.parameter().s(), x, s);

        if (spline != null)
            return spline.entry(s);

        new Throwable().printStackTrace();
        throw new IllegalStateException();
    }

    /**
     * Samples the entire path evenly by distance. Since the spline parameterizer
     * now contains a pathwise distance limit, you shouldn't need this anymore.
     * 
     * One difference is that the bisection method doesn't provide even sampling at
     * all: if round N is almost dense enough, then round N+1 can be almost twice as
     * dense as that. Maybe that's ok.
     */
    static PathSE2Point[] resample(PathSE2 p, double step) {
        double maxDistance = p.distance(p.length() - 1);
        if (maxDistance == 0)
            throw new IllegalArgumentException("max distance must be greater than zero");
        int num_states = (int) Math.ceil(maxDistance / step) + 1;
        if (DEBUG)
            System.out.printf("resample max distance %f step %f num states %d f %f\n",
                    maxDistance, step, num_states, maxDistance / step);
        PathSE2Point[] samples = new PathSE2Point[num_states];
        for (int i = 0; i < num_states; ++i) {
            // the dtheta and curvature come from here and are never changed.
            // the values here are just interpolated from the original values.
            double d = Math.min(i * step, maxDistance);
            PathSE2Point state = PathUtil.sample(p, d);
            if (state == null)
                continue;
            if (DEBUG)
                System.out.printf("RESAMPLE: i=%d d=%f state=%s\n", i, d, state);
            samples[i] = state;
        }
        return samples;
    }

    /**
     * Walks through all the points to find the bracketing points, and then
     * interpolates between them.
     * 
     * Beware, can return null if the path is empty.
     * 
     * This is not useful for operation, maybe useful for visualization; the path
     * should have enough states so you can just look at them directly.
     * 
     * @param distance in meters, always a non-negative number.
     */
    public static PathSE2Point sample(PathSE2 p, double distance) {
        // if an interpolated point is more than this far from an endpoint,
        // it indicates the endpoints are too far apart, including too far apart
        // in rotation, which is an aspect of the path scheduling that the
        // scheduler can't see
        // TODO: make this a constructor parameter.
        double INTERPOLATION_LIMIT = 0.3;
        if (distance >= p.distance(p.length() - 1)) {
            // off the end
            return p.getEntry(p.length() - 1).point();
        }
        if (distance <= 0.0) {
            // before the start
            return p.getEntry(0).point();
        }
        for (int i0 = 0; i0 < p.length() - 1; ++i0) {
            // walk through the points to bracket the desired distance
            int i1 = i0 + 1;
            PathSE2Entry e0 = p.getEntry(i0);
            PathSE2Point p0 = e0.point();
            PathSE2Entry e1 = p.getEntry(i1);
            PathSE2Point p1 = e1.point();
            double d0 = p.m_distances[i0];
            double d1 = p.m_distances[i1];
            double d = d1 - d0;
            if (d1 >= distance) {
                // Found the bracket.
                double s = (distance - d0) / d;
                PathSE2Entry lerp = interpolate(e0, e1, s);
                // disallow corners
                Twist2d t0 = p0.waypoint().course().minus(lerp.point().waypoint().course());
                double l0 = Metrics.l2Norm(t0);
                Twist2d t1 = p1.waypoint().course().minus(lerp.point().waypoint().course());
                double l1 = Metrics.l2Norm(t1);
                if (Math.max(l0, l1) > INTERPOLATION_LIMIT)
                    System.out.printf(
                            "WARNING!  Interpolated value too far away, p0=%s, p1=%s, t0=%s t1=%s.  This usually indicates a sharp corner in the path, which is not allowed.",
                            p0, p1, t0, t1);
                return lerp.point();
            }
        }
        return null;
    }

}
