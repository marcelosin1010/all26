package org.team100.lib.util;

/**
 * https://www.geeksforgeeks.org/maths/quadratic-interpolation/
 * https://en.wikipedia.org/wiki/Polynomial_interpolation
 * https://en.wikipedia.org/wiki/Lagrange_polynomial
 */
public class QuadraticInterpolator {
    /**
     * Given three points, return y for the given x
     */
    public static double interpolate(
            double x0, double y0,
            double x1, double y1,
            double x2, double y2,
            double x) {
        // Lagrange bases:
        double L0 = (x - x1) * (x - x2) / ((x0 - x1) * (x0 - x2));
        double L1 = (x - x0) * (x - x2) / ((x1 - x0) * (x1 - x2));
        double L2 = (x - x0) * (x - x1) / ((x2 - x0) * (x2 - x1));
        // Lagrange polynomial;
        return y0 * L0 + y1 * L1 + y2 * L2;
    }

}
