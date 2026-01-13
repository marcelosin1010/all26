package org.team100.lib.trajectory;

import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

/**
 * Stuff about curves.
 * 
 * A curve is the image of a path, independent of parameterization.
 */
public class CurveUtil {
    private static final boolean DEBUG = false;

    /**
     * Unit tangent vector.
     * 
     * See MATH.md.
     *
     * @param rprime position derivative with respect to any parameterization
     */
    public static <N extends Num> Vector<N> T(Vector<N> rprime) {
        double rprimenorm = rprime.norm();
        Vector<N> T = rprime.div(rprimenorm);
        return T;
    }

    /**
     * In R2, the scalar curvature, $\kappa$, is the norm of the curvature vector
     * (K), signed (CCW positive) with respect to the unit tangent vector (T).
     * 
     * see MATH.md.
     * 
     * https://en.wikipedia.org/wiki/Curvature
     * 
     * TODO: move this to something like PathUtil since it's not about splines.
     */
    public static double kappaSigned(Vector<N2> T, Vector<N2> K) {
        // which direction is K from T?
        double det = T.get(0) * K.get(1) - T.get(1) * K.get(0);
        return K.norm() * Math.signum(det);
    }

    /**
     * Curvature vector in any dimensionality.
     * 
     * Note for a spline in SE(n), the curvature describes the path in Rn, i.e. the
     * rotational part of the spline is not included.
     * 
     * See MATH.md.
     * 
     * @param rprime      position derivative with respect to any parameterization
     * @param rprimeprime second derivative
     */
    public static <N extends Num> Vector<N> K(Vector<N> rprime, Vector<N> rprimeprime) {
        if (DEBUG)
            System.out.printf("rprime %s rprimeprime %s\n",
                    StrUtil.vecStr(rprime), StrUtil.vecStr(rprimeprime));
        Vector<N> T = T(rprime);
        if (DEBUG)
            System.out.printf("T %s\n", StrUtil.vecStr(T));
        double rprimenorm = rprime.norm();
        // when rprimenorm is zero, curvature is meaningless, return zero.
        if (Math.abs(rprimenorm) < 1e-6) {
            return rprime.times(0);
        }
        Vector<N> p2 = rprimeprime.div(rprimenorm * rprimenorm);
        Vector<N> K = p2.minus(T.times(T.dot(p2)));
        if (DEBUG)
            System.out.printf("K %s\n", StrUtil.vecStr(K));
        return K;
    }

}
