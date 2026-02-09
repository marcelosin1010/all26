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
    private static final IsotropicNoiseSE2 DEFAULT_STATE_STDDEV = IsotropicNoiseSE2.fromStdDev(0.1, 0.1);

    /**
     * This value is tuned so that errors scale at 0.2x per second. See
     * SwerveDrivePoseEstimator100Test::testFirmerNudge.
     * TODO: get rid of this
     */
    static final IsotropicNoiseSE2 TIGHT_STATE_STDDEV = IsotropicNoiseSE2.fromStdDev(0.001, 0.1);

    /**
     * Standard deviation of vision updates in SE(2).
     * 
     * Data comes from eyeballing the graphs in the Wang paper.
     * 
     * The vision system is good at estimating range from the tag, but it is not
     * very good at estimating bearing from the tag. The large uncertainty in
     * bearing results in a crescent-shaped uncertainty in the robot position. We
     * don't have a way to represent that shape, so instead we just take the
     * maximum.
     * 
     * Note: due to the uncertainty singularity on the tag axis, when we are
     * directly in front of the tag, we can't use it at all.
     */
    static IsotropicNoiseSE2 visionMeasurementStdDevs(double distanceM, double offAxisAngleRad) {
        if (distanceM < 0)
            throw new IllegalArgumentException();
        if (offAxisAngleRad < 0)
            throw new IllegalArgumentException();
        // these extra 0.01 values are total guesses
        // TODO: calibrate this, remove it?
        double cartesianErrorM = figure5(distanceM) + 0.01;
        double rotationErrorRad = figure6(offAxisAngleRad) + 0.01;
        double rotationEffectM = distanceM * rotationErrorRad;
        double maxCartesian = Math.max(cartesianErrorM, rotationEffectM);
        return IsotropicNoiseSE2.fromStdDev(maxCartesian, rotationErrorRad);
    }

    /**
     * Figure 5 in the Wang paper (below 15 m) indicates a linear relationship
     * between cartesian error and tag distance.
     *
     * TODO: calibrate this
     */
    static double figure5(double distanceM) {
        return 0.03 * distanceM;
    }

    /**
     * Figure 6 in the Wang paper indicates a U-shaped relationship between the "off
     * axis" angle and the error. Note the infinite error at zero.
     * 
     * Remember that our tag normal direction is "into the page", but the "off axis"
     * normal is "out of the page".
     * 
     * TODO: calibrate this
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
    static IsotropicNoiseSE2 stateStdDevs() {
        if (Experiments.instance.enabled(Experiment.AvoidVisionJitter)) {
            return TIGHT_STATE_STDDEV;
        }
        return DEFAULT_STATE_STDDEV;
    }

    /**
     * The error in odometry is superlinear in speed. Since the odometry samples
     * happen regularly, we can use the sample distance as a measure of speed.
     * 
     * I completely made this up
     * https://docs.google.com/spreadsheets/d/1DmHL1UDd6vngmr-5_9fNHg2xLC4TEVWTN2nHZBOnje0/edit?gid=995645441#gid=995645441
     */
    static IsotropicNoiseSE2 odometryStdDevs(double distanceM, double rotationRad) {
        // We kinda measured 5% error in the best (slow) case.
        double lowSpeedError = 0.05;
        // This is just a guess
        double superError = 0.5;
        double cartesian = lowSpeedError * distanceM + superError * distanceM * distanceM;
        // We haven't measured this, so just guess it's the same???
        double rotation = lowSpeedError * rotationRad + superError * rotationRad * rotationRad;
        return IsotropicNoiseSE2.fromStdDev(cartesian, rotation);

    }

    /**
     * TODO: remove this
     * @param stateNoise  state noise in SE(2)
     * @param visionNoise vision update noise in SE(2)
     * @param twist       from sample to measurement, i.e. in the intrinsic frame of
     *                    the sample.
     */
    static Twist2d getScaledTwist(
            IsotropicNoiseSE2 stateNoise,
            IsotropicNoiseSE2 visionNoise,
            Twist2d twist) {
        double cartesianWeight = cartesianWeight(stateNoise, visionNoise);
        double rotationWeight = rotationWeight(stateNoise, visionNoise);
        return new Twist2d(
                cartesianWeight * twist.dx,
                cartesianWeight * twist.dy,
                rotationWeight * twist.dtheta);
    }

    /**
     * Use inverse-variance weighting to scale the delta.
     * 
     * @param stateNoise  state noise in SE(2)
     * @param visionNoise vision update noise in SE(2)
     * @param delta       from sample to measurement, in (extrinsic) field frame.
     */
    static DeltaSE2 getScaledDelta(
            IsotropicNoiseSE2 stateNoise,
            IsotropicNoiseSE2 visionNoise,
            DeltaSE2 delta) {
        double cartesianWeight = cartesianWeight(stateNoise, visionNoise);
        double rotationWeight = rotationWeight(stateNoise, visionNoise);
        return new DeltaSE2(
                delta.getTranslation().times(cartesianWeight),
                delta.getRotation().times(rotationWeight));
    }

    /**
     * Weights for the vision update, using inverse-variance weighting.
     * 
     * @param stateNoise  state noise in SE(2)
     * @param visionNoise vision update noise in SE(2)
     * @return update weight for cartesian dimensions
     */
    static double cartesianWeight(
            IsotropicNoiseSE2 stateNoise,
            IsotropicNoiseSE2 visionNoise) {
        return weight(
                stateNoise.cartesianVariance(),
                visionNoise.cartesianVariance());
    }

    /**
     * Weights for the vision update, using inverse-variance weighting.
     * 
     * @param stateNoise  state noise in SE(2)
     * @param visionNoise vision update noise in SE(2)
     * @return update weight for rotation
     */
    static double rotationWeight(
            IsotropicNoiseSE2 stateNoise,
            IsotropicNoiseSE2 visionNoise) {
        return weight(
                stateNoise.rotationVariance(),
                visionNoise.rotationVariance());
    }

    /**
     * Weight by inverse variance, for use in a weighted sum of means.
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
     * @param var1  state variance
     * @param var2  update variance
     * @param state weight of the update, in the range [0, 1]
     */
    static double weight(double var1, double var2) {
        if (var1 < 1e-6)
            return 0;
        if (var2 < 1e-6)
            return 1;
        return (1 / var2) / (1 / var1 + 1 / var2);
    }

    /**
     * Variance of the weighted mean.
     * 
     * This is simply the reciprocal of the sum of the reciprocals.
     * 
     * https://en.wikipedia.org/wiki/Inverse-variance_weighting
     * 
     * @param var1 state variance
     * @param var2 update variance
     * @return variance of the weighted mean.
     */
    static double variance(double var1, double var2) {
        return 1 / (1 / var1 + 1 / var2);
    }
}
