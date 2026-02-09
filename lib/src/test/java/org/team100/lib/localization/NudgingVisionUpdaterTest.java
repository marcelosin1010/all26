package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class NudgingVisionUpdaterTest {
    private static final double DELTA = 0.001;

    /**
     * If the measurement is the same as the sample, nothing happens.
     */
    @Test
    void testZeroNudge() {
        // state is twice as confident as camera
        IsotropicNoiseSE2 stateStdDev = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        IsotropicNoiseSE2 visionStdDev = IsotropicNoiseSE2.fromStdDev(0.02, 0.02);
        final Pose2d sample = new Pose2d();
        final Pose2d measurement = new Pose2d();
        final Pose2d nudged = NudgingVisionUpdater.nudge(sample, measurement, stateStdDev, visionStdDev);
        assertEquals(0, nudged.getX(), 1e-6);
        assertEquals(0, nudged.getY(), 1e-6);
        assertEquals(0, nudged.getRotation().getRadians(), 1e-6);

    }

    /**
     * Let's say the camera is correct, and it says the sample is 0.1 meters off.
     * How long does it take for the pose buffer to contain the right pose? Kind of
     * a long time.
     */
    @Test
    void testGentleNudge() {
        int frameRate = 50;
        IsotropicNoiseSE2 stateStdDev = IsotropicNoiseSE2.fromStdDev(0.001, 0.1);
        IsotropicNoiseSE2 visionStdDev = IsotropicNoiseSE2.fromStdDev(0.1, Double.MAX_VALUE);
        final Pose2d sample = new Pose2d();
        final Pose2d measurement = new Pose2d(0.1, 0, new Rotation2d());
        Pose2d nudged = sample;
        for (int i = 0; i < frameRate; ++i) {
            nudged = NudgingVisionUpdater.nudge(nudged, measurement, stateStdDev, visionStdDev);
        }
        // after 1 sec the error is about 6 cm which is too slow.
        Transform2d error = measurement.minus(nudged);
        assertEquals(0.099, error.getX(), DELTA);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);
        //
        for (int i = 0; i < frameRate; ++i) {
            nudged = NudgingVisionUpdater.nudge(nudged, measurement, stateStdDev, visionStdDev);
        }
        // after 2 sec the error is about 4 cm.
        error = measurement.minus(nudged);
        assertEquals(0.099, error.getX(), DELTA);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);
    }

    /**
     * Same as above with firmer updates. We want error under 2cm within about 1s.
     */
    @Test
    void testFirmerNudge() {
        int frameRate = 50;
        IsotropicNoiseSE2 stateStdDev = IsotropicNoiseSE2.fromStdDev(0.001, 0.1);
        IsotropicNoiseSE2 visionStdDev = IsotropicNoiseSE2.fromStdDev(0.03, Double.MAX_VALUE);
        final Pose2d sample = new Pose2d();
        final Pose2d measurement = new Pose2d(0.1, 0, new Rotation2d());
        Pose2d nudged = sample;
        for (int i = 0; i < frameRate; ++i) {
            nudged = NudgingVisionUpdater.nudge(nudged, measurement, stateStdDev, visionStdDev);
        }
        // after 1 sec the error is about 2 cm which is the target.
        Transform2d error = measurement.minus(nudged);
        assertEquals(0.094, error.getX(), DELTA);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);
        //
        for (int i = 0; i < frameRate; ++i) {
            nudged = NudgingVisionUpdater.nudge(nudged, measurement, stateStdDev, visionStdDev);
        }
        // after 2 sec the error is about 4 mm which seems plenty tight
        error = measurement.minus(nudged);
        assertEquals(0.089, error.getX(), DELTA);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);

    }

}
