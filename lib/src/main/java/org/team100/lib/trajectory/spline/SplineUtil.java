package org.team100.lib.trajectory.spline;

import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N2;

public class SplineUtil {
    private static final boolean DEBUG = false;

    /**
     * Position of offset for parameter, s.
     * 
     * The offset is a fixed length behind the toolpoint.
     */
    public static Vector<N2> offsetR(SplineSE2 spline, double l, double s) {
        Pose2d pose = spline.pose(s);
        // negative length here because the offset is "behind" the toolpoint.
        Pose2d p2 = pose.transformBy(new Transform2d(-l, 0, new Rotation2d()));
        return VecBuilder.fill(p2.getX(), p2.getY());
    }

    /**
     * Derivative of offset with respect to parameter, s.
     * 
     * Since the offset length is fixed, the only velocity is tangential, from the
     * heading rate.
     */
    public static Vector<N2> offsetRprime(SplineSE2 spline, double l, double s) {
        Pose2d pose = spline.pose(s);
        Rotation2d radial = pose.getRotation().plus(Rotation2d.k180deg);
        Rotation2d tangential = radial.plus(Rotation2d.kCCW_Pi_2);
        // dtheta/ds
        double headingRate = spline.dheading(s);
        // dx/ds = dtheta/ds * x/theta
        double tangentialVelocity = l * headingRate;
        return VecBuilder.fill(tangentialVelocity * tangential.getCos(), tangentialVelocity * tangential.getSin());
    }

    /**
     * Second derivative of offset with respect to parameter, s.
     * 
     * The offset point acceleration has two components: tangential and centripetal.
     */
    public static Vector<N2> offsetRprimeprime(SplineSE2 spline, double l, double s) {
        Pose2d pose = spline.pose(s);
        Rotation2d radial = pose.getRotation().plus(Rotation2d.k180deg);
        Rotation2d tangential = radial.plus(Rotation2d.kCCW_Pi_2);

        // centripetal a = r omega^2
        // dtheta/ds
        double headingRate = spline.dheading(s);
        // d^2x/ds^2 = x/theta * dtheta/ds * dtheta/ds
        double ca = l * headingRate * headingRate;
        // tangential
        double headingAccel = spline.ddheading(s);
        double ta = l * headingAccel;

        // note inversion of radial, it's pointing towards the center
        return VecBuilder.fill(
                -ca * radial.getCos() + ta * tangential.getCos(),
                -ca * radial.getSin() + ta * tangential.getSin());
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
     * TODO: move this to something like PathUtil since it's not about splines.
     */
    public static double kappaSigned(Vector<N2> T, Vector<N2> K) {
        // which direction is K from T?
        double det = T.get(0) * K.get(1) - T.get(1) * K.get(0);
        return K.norm() * Math.signum(det);
    }

}
