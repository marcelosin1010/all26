package org.team100.lib.targeting;

import java.util.function.DoubleFunction;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Lookup shooting parameters for a given target range
 */
public class InverseRange implements DoubleFunction<FiringParameters> {
    private static final boolean DEBUG = true;

    /**
     * Precomputation lower bound.
     * 
     * TODO: probably for real robotics we want the inidirect solution.
     */
    private static final double MIN_ELEVATION = 0.0;
    /**
     * Precomputation upper bound.
     * 
     * This is low so that the test results match the shooting method, which prefers
     * the direct solution.
     * 
     * TODO: probably for real robotics we want the indirect solution.
     */
    private static final double MAX_ELEVATION = 0.6;
    /** Precomputation step. */
    private static final double ELEVATION_STEP = 0.01;

    /** key = distance in meters, value = solution */
    private final InterpolatingTreeMap<Double, FiringParameters> m_map;

    public InverseRange(Drag d, double targetHeight, double v, double omega) {
        RangeSolver rangeSolver = new RangeSolver(d, targetHeight);
        m_map = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(), new FiringParametersInterpolator());
        if (DEBUG)
            System.out.println("range, elevation, tof");
        for (double elevation = MIN_ELEVATION; elevation <= MAX_ELEVATION; elevation += ELEVATION_STEP) {
            FiringSolution solution = rangeSolver.getSolution(v, omega, elevation);
            if (solution == null) {
                // no solution
                continue;
            }
            FiringParameters params = new FiringParameters(elevation, solution.tof());
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f, %6.3f\n",
                        solution.range(), params.elevation(), params.tof());
            m_map.put(solution.range(), params);
        }
    }

    public FiringParameters apply(double range) {
        return m_map.get(range);
    }

}
