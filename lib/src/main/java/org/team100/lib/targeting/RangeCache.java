package org.team100.lib.targeting;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Precompute firing solutions.
 */
public class RangeCache implements IRange {
    private static final boolean DEBUG = false;
    /** Precomputation lower bound. */
    private static final double MIN_ELEVATION = 0;
    /** Precomputation upper bound. */
    private static final double MAX_ELEVATION = Math.PI / 2;
    /** Precomputation step. */
    private static final double ELEVATION_STEP = 0.01;

    /** key = elevation in radians, value = solution */
    private final InterpolatingTreeMap<Double, FiringSolution> m_map;

    /**
     * @param rangeSolver solver with drag and velocity
     * @param omega       spin in rad/s, positive is backspin
     */
    public RangeCache(RangeSolver rangeSolver, double v, double omega) {
        InterpolatingTreeMap<Double, FiringSolution> map = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(), new FiringSolutionInterpolator());
        if (DEBUG)
            System.out.println("elevation, range, tof");
        for (double elevation = MIN_ELEVATION; elevation <= MAX_ELEVATION; elevation += ELEVATION_STEP) {
            FiringSolution solution = rangeSolver.getSolution(v, omega, elevation);
            if (solution == null) {
                // no solution
                continue;
            }
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f, %6.3f\n",
                        elevation, solution.range(), solution.tof());
            map.put(elevation, solution);
        }
        m_map = map;
    }

    /**
     * @param elevation in radians
     */
    @Override
    public FiringSolution get(double elevation) {
        return m_map.get(elevation);
    }

}
