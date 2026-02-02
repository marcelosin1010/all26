package org.team100.lib.localization;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.DeltaSE2;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * Methods governing vision update uncertainties.
 * 
 * https://april.eecs.umich.edu/media/media/pdfs/wang2016iros.pdf
 * https://docs.google.com/spreadsheets/d/1StMbOyksydpzFmpHbZBL7ICMQtHnOBubrOMeYSx0M6E
 */
public class Uncertainty {
    /**
     * this is the default value which, in hindsight, seems ridiculously high.
     * TODO: do some calibration of this
     */
    private static final IsotropicSigmaSE2 DEFAULT_STATE_STDDEV = new IsotropicSigmaSE2(0.1, 0.1);

    /**
     * This value is tuned so that errors scale at 0.2x per second. See
     * SwerveDrivePoseEstimator100Test::testFirmerNudge.
     * TODO: get rid of this
     */
    static final IsotropicSigmaSE2 TIGHT_STATE_STDDEV = new IsotropicSigmaSE2(0.001, 0.1);

    /**
     * Standard deviation of vision updates in SE(2).
     * 
     * Data comes from eyeballing the graphs in the Wang paper.
     * 
     * Note that rotational uncertainty should affect translational uncertainty too:
     * the tag uncertainty model is polar.
     */
    static IsotropicSigmaSE2 visionMeasurementStdDevs(double distanceM, double offAxisAngleRad) {
        // these extra 0.01 values are total guesses
        // TODO: remove them?
        double cartesianErrorM = figure5(distanceM) + 0.01;
        double rotationErrorRad = figure6(offAxisAngleRad) + 0.01;
        // TODO: add effect of rotation error on cartesian error
        return new IsotropicSigmaSE2(cartesianErrorM, rotationErrorRad);
    }

    /**
     * Figure 5 in the Wang paper (below 15 m) indicates a linear relationship
     * between cartesian error and tag distance.
     */
    static double figure5(double distanceM) {
        return 0.03 * distanceM;
    }

    /**
     * Figure 6 in the Wang paper indicates a U-shaped relationship between the "off
     * axis" angle and the error. Note the very large error at zero.
     * 
     * Remember that our tag normal direction is "into the page", but the "off axis"
     * normal is "out of the page".
     */
    static double figure6(double offAxisAngleRad) {
        if (offAxisAngleRad < 0)
            throw new IllegalArgumentException("angle must be non-negative");
        // This uses degrees because figure 6 uses degrees.
        double offAxisDegrees = Math.toDegrees(offAxisAngleRad);
        if (offAxisDegrees < 3)
            return Double.MAX_VALUE;
        double errorDeg = 10 / offAxisDegrees + 10 / Math.pow(85 - offAxisDegrees, 1.2);
        return Math.toRadians(errorDeg);
    }

    /**
     * Standard devation of robot state in SE(2). Note this is just a 3 vector; the
     * covariances are zero. We *could* use covariances here (so the shape of the
     * error could be extended along a diagonal), but it would be more complicated
     * for little benefit.
     */
    static IsotropicSigmaSE2 stateStdDevs() {
        if (Experiments.instance.enabled(Experiment.AvoidVisionJitter)) {
            return Uncertainty.TIGHT_STATE_STDDEV;
        }
        return Uncertainty.DEFAULT_STATE_STDDEV;
    }

    /**
     * @param stateSigma  standard deviation of state in SE(2)
     * @param visionSigma standard deviation of vision update in SE(2)
     * @param twist       from sample to measurement, i.e. in the intrinsic frame of
     *                    the sample.
     */
    static Twist2d getScaledTwist(
            IsotropicSigmaSE2 stateSigma,
            IsotropicSigmaSE2 visionSigma,
            Twist2d twist) {
        double cartesianWeight = Uncertainty.cartesianWeight(stateSigma, visionSigma);
        double rotationWeight = Uncertainty.rotationWeight(stateSigma, visionSigma);
        return new Twist2d(
                cartesianWeight * twist.dx,
                cartesianWeight * twist.dy,
                rotationWeight * twist.dtheta);
    }

    /**
     * @param stateSigma  standard deviation of state in SE(2)
     * @param visionSigma standard deviation of vision update in SE(2)
     * @param delta       from sample to measurement, in (extrinsic) field frame.
     */
    static DeltaSE2 getScaledDelta(
            IsotropicSigmaSE2 stateSigma,
            IsotropicSigmaSE2 visionSigma,
            DeltaSE2 delta) {
        double cartesianWeight = Uncertainty.cartesianWeight(stateSigma, visionSigma);
        double rotationWeight = Uncertainty.rotationWeight(stateSigma, visionSigma);
        return new DeltaSE2(
                delta.getTranslation().times(cartesianWeight),
                delta.getRotation().times(rotationWeight));
    }

    /**
     * Weights for the vision update, using inverse-variance weighting.
     * 
     * This used to use inverse-standard-deviation weighting, which is just wrong.
     * 
     * @param stateSigma  standard deviation of state in SE(2)
     * @param visionSigma standard deviation of vision update in SE(2)
     * @return update weight for cartesian dimensions
     */
    static double cartesianWeight(
            IsotropicSigmaSE2 stateSigma,
            IsotropicSigmaSE2 visionSigma) {
        return inverseVarianceWeighting(
                Math.pow(stateSigma.cartesian(), 2),
                Math.pow(visionSigma.cartesian(), 2));
    }

    /**
     * Weights for the vision update, using inverse-variance weighting.
     * 
     * This used to use inverse-standard-deviation weighting, which is just wrong.
     * 
     * @param stateSigma  standard deviation of state in SE(2)
     * @param visionSigma standard deviation of vision update in SE(2)
     * @return update weight for rotation
     */
    static double rotationWeight(
            IsotropicSigmaSE2 stateSigma,
            IsotropicSigmaSE2 visionSigma) {
        return inverseVarianceWeighting(
                Math.pow(stateSigma.rotation(), 2),
                Math.pow(visionSigma.rotation(), 2));
    }

    /**
     * Simple weighting by inverse variance.
     * 
     * Inverse-variance weighting is the "optimal" (i.e. maximum-likelihood) thing
     * to do if the variables are independent and normal. They are neither, but we
     * use this anyway.
     * 
     * The previous mixer weighted by standard deviation, which was just wrong.
     * 
     * https://en.wikipedia.org/wiki/Inverse-variance_weighting
     * https://en.wikipedia.org/wiki/Weighted_arithmetic_mean
     * https://www.nist.gov/system/files/documents/2017/05/09/combine-1.pdf
     * 
     * @param q     state variance
     * @param r     vision variance
     * @param state weight of the update, in the range [0, 1]
     */
    static double inverseVarianceWeighting(double q, double r) {
        if (q < 1e-6)
            return 0;
        if (r < 1e-6)
            return 1;
        return (1 / r) / (1 / q + 1 / r);
    }
}
