package org.team100.lib.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.system.NumericalIntegration;

/**
 * Given initial conditions of elevation and fixed muzzle velocity, integrates
 * the drag model until position is less than zero.
 * 
 * Returns a firing solution (range and time of flight).
 */
public class RangeSolver {

    private static final boolean DEBUG = false;

    /**
     * RK4 integration with this resolution.
     * 
     * See RangeSolverTest for choice of DT
     */
    static final double INTEGRATION_DT = 0.001;

    private final Drag m_d;
    private final double m_targetHeight;

    /**
     * @param targetHeight Height of the target above the firing height (not the
     *                     floor)
     */
    public RangeSolver(Drag d, double targetHeight) {
        m_d = d;
        m_targetHeight = targetHeight;
    }

    /**
     * Both range and time-of-flight are always slight underestimates.
     * 
     * @param d         drag model
     * @param v         muzzle speed in m/s
     * @param omega     spin in rad/s, positive is backspin
     * @param elevation in rad
     */
    public FiringSolution getSolution(
            double v, double omega, double elevation) {
        return solveWithDt(v, omega, elevation, INTEGRATION_DT);
    }

    /** Package-private for testing */
    FiringSolution solveWithDt(
            double v, double omega, double elevation, double dt) {
        if (dt < 1e-6)
            throw new IllegalArgumentException("must use nonzero dt");
        double vx = v * Math.cos(elevation);
        double vy = v * Math.sin(elevation);
        // state is (x, y, theta, vx, vy, omega)
        Matrix<N6, N1> x = VecBuilder.fill(0, 0, 0, vx, vy, omega);
        Matrix<N6, N1> prevX = x;
        double t = 0;
        for (t = 0; t < 10; t += dt) {
            // this is the x for t+dt.
            x = NumericalIntegration.rk4(m_d, prevX, dt);
            double range = x.get(0, 0);
            double height = x.get(1, 0);
            double prevRange = prevX.get(0, 0);
            double prevHeight = prevX.get(1, 0);

            double dy = height - prevHeight;
            if (DEBUG)
                System.out.printf("t %f prevRange %f range %f prevHeight %f height %f dy %f\n",
                        t, prevRange, range, prevHeight, height, dy);
            // on the way down, and below the target height
            if (dy < 0 && height < m_targetHeight) {
                if (DEBUG)
                    System.out.println("impact");
                // interpolate using the floor position
                // more-clever interpolation or integration makes no difference.

                double lerp = -1.0 * prevHeight / dy;
                double drange = range - prevRange; // a positive number
                double rangeLerp = MathUtil.interpolate(prevRange, range, lerp);
                double tofLerp = t + dt * lerp;
                // to compute the target elevation, look at the last two points.
                if (DEBUG)
                    System.out.printf("prevRange %f range %f prevHeight %f height %f\n",
                            prevRange, range, prevHeight, height);
                double targetElevation = Math.atan2(-1.0 * dy, drange);
                if (DEBUG)
                    System.out.printf("e %f\n", targetElevation);
                return new FiringSolution(rangeLerp, tofLerp, targetElevation);
            }
            prevX = x;
        }
        // if we got to the end, there's no (useful) solution.
        return null;
    }
}
