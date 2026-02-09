package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DeltaSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class UncertaintyTest {
    private static final double DELTA = 0.001;

    @Test
    void testFigure5() {
        assertEquals(0.03, Uncertainty.figure5(1), DELTA);
        assertEquals(0.00, Uncertainty.figure5(0), DELTA);
    }

    @Test
    void testFigure6() {
        assertEquals(Math.toRadians(0.342), Uncertainty.figure6(Math.PI / 4), DELTA);
        assertEquals(Double.MAX_VALUE, Uncertainty.figure6(0), DELTA);
    }

    @Test
    void testVisionStdDevs() {
        double targetRangeM = 1.0;
        IsotropicNoiseSE2 visionStdDev = Uncertainty.visionMeasurementStdDevs(targetRangeM, 0.1);
        assertEquals(0.041, visionStdDev.cartesian(), DELTA);
        assertEquals(0.041, visionStdDev.rotation(), DELTA);
    }

    @Test
    void testStateStdDevs() {
        // these are the "antijitter" values.
        // 1 mm, very low
        IsotropicNoiseSE2 stateStdDev = Uncertainty.TIGHT_STATE_STDDEV;
        assertEquals(0.001, stateStdDev.cartesian(), DELTA);
        assertEquals(0.1, stateStdDev.rotation(), DELTA);
    }

    @Test
    void testOdometry() {
        IsotropicNoiseSE2 odo = Uncertainty.odometryStdDevs(0, 0);
        assertEquals(0.0, odo.cartesian(), 1e-6);
        assertEquals(0.0, odo.rotation(), 1e-6);
        // moving pretty slowly, 0.5 m/s over 0.02 sec
        odo = Uncertainty.odometryStdDevs(0.01, 0);
        // about 3 cm/s of error, which seems reasonable
        assertEquals(0.00055, odo.cartesian(), 1e-6);
        assertEquals(0.0, odo.rotation(), 1e-6);
        // moving pretty fast, 5 m/s over 0.02 sec
        odo = Uncertainty.odometryStdDevs(0.1, 0);
        // 0.5 m/s of error, a whole lot!
        assertEquals(0.010, odo.cartesian(), 1e-6);
        assertEquals(0.0, odo.rotation(), 1e-6);
        // this is slow rotation, 1 rad/s over 0.02 sec
        odo = Uncertainty.odometryStdDevs(0, 0.02);
        assertEquals(0.0, odo.cartesian(), 1e-3);
        // 0.06 rad/s of error, a few degrees
        assertEquals(0.0012, odo.rotation(), 1e-3);
        // this is very fast rotation, 10 rad/s over 0.02 sec
        odo = Uncertainty.odometryStdDevs(0, 0.2);
        assertEquals(0.0, odo.cartesian(), 1e-3);
        // 1.5 rad/s of error, that's a whole lot!
        assertEquals(0.0300, odo.rotation(), 1e-3);
    }

    @Test
    void testTwistVsDelta() {
        // we're using an SE(2) twist to represent the difference between the state and
        // the measurement, and then we scale that twist. But I think that might not be
        // the right thing.
        Pose2d state = new Pose2d();
        Pose2d measurement = new Pose2d(1, 0, new Rotation2d(1));
        Twist2d twist = state.log(measurement);
        double scale = 0.5;
        Twist2d scaledTwist = new Twist2d(scale * twist.dx, scale * twist.dy, scale * twist.dtheta);
        Pose2d result = state.exp(scaledTwist);
        // I would expect the scaling to affect each dimension separately, but that's
        // not what happens.
        assertEquals(0.5000, result.getX(), DELTA);
        assertEquals(-0.128, result.getY(), DELTA);
        assertEquals(0.5000, result.getRotation().getRadians(), DELTA);
        DeltaSE2 delta = DeltaSE2.delta(state, measurement);
        DeltaSE2 scaledDelta = delta.times(scale);
        Pose2d result2 = scaledDelta.plus(state);
        // The delta just applies each dimension separately.
        assertEquals(0.5, result2.getX(), DELTA);
        assertEquals(0.0, result2.getY(), DELTA);
        assertEquals(0.5, result2.getRotation().getRadians(), DELTA);
    }

    @Test
    void testScaledTwist() {
        IsotropicNoiseSE2 stateStdDev = IsotropicNoiseSE2.fromStdDev(0.02, 0.02);
        double targetRangeM = 1.0;
        double offAxisRad = 1.0;
        IsotropicNoiseSE2 visionStdDev = Uncertainty.visionMeasurementStdDevs(targetRangeM, offAxisRad);
        assertEquals(0.040, visionStdDev.cartesian(), DELTA);
        assertEquals(0.016, visionStdDev.rotation(), DELTA);
        // 10 cm of difference between the vision update and the current pose
        Twist2d twist = new Twist2d(0.1, 0.1, 0.1);
        Twist2d scaled = Uncertainty.getScaledTwist(stateStdDev, visionStdDev, twist);
        // difference is discounted 20x
        assertEquals(0.02, scaled.dx, DELTA);
        assertEquals(0.02, scaled.dy, DELTA);
        assertEquals(0.06, scaled.dtheta, DELTA);
    }

    @Test
    void testScaledDelta() {
        IsotropicNoiseSE2 stateStdDev = IsotropicNoiseSE2.fromStdDev(0.02, 0.02);
        double targetRangeM = 1.0;
        double offAxisRad = 1.0;
        IsotropicNoiseSE2 visionStdDev = Uncertainty.visionMeasurementStdDevs(targetRangeM, offAxisRad);
        assertEquals(0.040, visionStdDev.cartesian(), DELTA);
        assertEquals(0.016, visionStdDev.rotation(), DELTA);
        // 10 cm of difference between the vision update and the current pose
        DeltaSE2 twist = new DeltaSE2(new Translation2d(0.1, 0.1), new Rotation2d(0.1));
        DeltaSE2 scaled = Uncertainty.getScaledDelta(stateStdDev, visionStdDev, twist);
        // difference is discounted 20x
        assertEquals(0.02, scaled.getX(), DELTA);
        assertEquals(0.02, scaled.getY(), DELTA);
        assertEquals(0.06, scaled.getRadians(), DELTA);
    }

    @Test
    void testCartesianWeight() {
        IsotropicNoiseSE2 stateStdDev = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        double targetRangeM = 1.0;
        double offAxisRad = 1.0;
        IsotropicNoiseSE2 visionStdDev = Uncertainty.visionMeasurementStdDevs(targetRangeM, offAxisRad);
        assertEquals(0.040, visionStdDev.cartesian(), DELTA);
        assertEquals(0.016, visionStdDev.rotation(), DELTA);
        double w = Uncertainty.cartesianWeight(stateStdDev, visionStdDev);
        assertEquals(0.059, w, DELTA);
    }

    @Test
    void testRotationWeight() {
        IsotropicNoiseSE2 stateStdDev =  IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        double targetRangeM = 1.0;
        double offAxisRad = 1.0;
        IsotropicNoiseSE2 visionStdDev = Uncertainty.visionMeasurementStdDevs(targetRangeM, offAxisRad);
        assertEquals(0.040, visionStdDev.cartesian(), DELTA);
        assertEquals(0.016, visionStdDev.rotation(), DELTA);
        double w = Uncertainty.rotationWeight(stateStdDev, visionStdDev);
        assertEquals(0.274, w, DELTA);
    }

    @Test
    void testWeight() {
        // very uncertain update is almost ignored
        assertEquals(0.010, Uncertainty.weight(1, 100), DELTA);
        assertEquals(0.091, Uncertainty.weight(1, 10), DELTA);
        // equal uncertainty yields equal weight
        assertEquals(0.5, Uncertainty.weight(1, 1), DELTA);
        assertEquals(0.910, Uncertainty.weight(10, 1), DELTA);
        // very confident update gets almost all the weight
        assertEquals(0.990, Uncertainty.weight(100, 1), DELTA);
    }

    @Test
    void testVariance() {
        // very uncertain update yields a confident result
        assertEquals(0.990, Uncertainty.variance(1, 100), DELTA);
        // less uncertain, result is tighter
        assertEquals(0.91, Uncertainty.variance(1, 10), DELTA);
        // two samples with same uncertainty -> twice as confident!
        assertEquals(0.5, Uncertainty.variance(1, 1), DELTA);
        assertEquals(0.91, Uncertainty.variance(10, 1), DELTA);
        // very confident update yields a confident result
        assertEquals(0.990, Uncertainty.variance(100, 1), DELTA);
    }

    @Test
    void testInvVarSweep() {
        // This is for gnuplot, to make a surface plot.
        // Put the output in data.dat, then run
        // splot 'data.dat'
        System.out.println("# q r mix\n");
        for (double q = 0; q < 0.1; q += 0.005) {
            for (double r = 0; r < 0.1; r += 0.005) {
                double mix = Uncertainty.weight(q, r);
                System.out.printf("%f %f %f\n", q, r, mix);
            }
        }
    }
}
