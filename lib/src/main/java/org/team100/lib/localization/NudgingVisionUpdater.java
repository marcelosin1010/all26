package org.team100.lib.localization;

import org.team100.lib.coherence.Takt;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Updates SwerveModelHistory with any vision input, by interpolating to find a
 * pose for the vision timestamp, nudging that pose towards the vision
 * measurement, and then asking the odometry updater to replay all the later
 * odometry.
 * 
 * The "nudging" here is essentially just a weighted average; you provide the
 * weights you want at update time.
 */
public class NudgingVisionUpdater implements VisionUpdater {

    private final SwerveHistory m_history;
    /** For replay. */
    private final OdometryUpdater m_odometryUpdater;
    /** To measure time since last update, for indicator. */
    private double m_latestTimeS;

    public NudgingVisionUpdater(
            SwerveHistory history,
            OdometryUpdater odometryUpdater) {
        m_history = history;
        m_odometryUpdater = odometryUpdater;
        m_latestTimeS = 0;
    }

    /**
     * Put a new state estimate based on the supplied pose. If not current,
     * subsequent wheel updates are replayed.
     * 
     * @param timestamp   When the measurement was made.
     * @param measurement Robot pose from vision.
     * @param visionNoise Measurement noise in SE(2).
     */
    @Override
    public void put(
            double timestamp,
            Pose2d measurement,
            IsotropicNoiseSE2 visionNoise) {

        // Skip too-old measurement
        if (m_history.tooOld(timestamp)) {
            return;
        }

        // Sample the history at the measurement time.
        SwerveState sample = m_history.getRecord(timestamp);

        // Nudge the sample pose towards the measurement.
        ModelSE2 sampleModel = sample.state();
        IsotropicNoiseSE2 sampleNoise = sample.noise();
        SwerveModulePositions samplePositions = sample.positions();

        Pose2d samplePose = sampleModel.pose();

        Pose2d nudged = nudge(samplePose, measurement, sampleNoise, visionNoise);

        // Vision updates do not affect the gyro measurement.
        Rotation2d sampleGyroYaw = sample.gyroYaw();

        // Vision updates do not affect the gyro bias estimate.
        VariableR1 sampleGyroBias = sample.gyroBias();

        // Vision updates to not affect the velocity estimate.
        VelocitySE2 sampleVelocity = sampleModel.velocity();

        ModelSE2 model = new ModelSE2(nudged, sampleVelocity);

        IsotropicNoiseSE2 noise = IsotropicNoiseSE2.inverseVarianceWeightedAverage(
                sampleNoise, visionNoise);

        // Remember the result.
        m_history.put(timestamp, model, noise, samplePositions, sampleGyroYaw, sampleGyroBias);

        // Replay everything after the sample.
        m_odometryUpdater.replay(timestamp);

        // Remember the time of this update.
        m_latestTimeS = Takt.get();
    }

    /**
     * The age of the last pose estimate, in seconds.
     * The caller could use this to, say, indicate tag visibility.
     */
    public double getPoseAgeSec() {
        return Takt.get() - m_latestTimeS;
    }

    /////////////////////////////////////////

    /**
     * Compute the weighted average of sample and measurement, using
     * inverse-variance weighting.
     * 
     * TODO: make an uncertain-pose type
     * 
     * @param sample      historical pose
     * @param measurement new input
     * @param stateSigma  standard deviation of the sample
     * @param visionSigma standard deviation of the measurement
     */
    static Pose2d nudge(
            Pose2d sample,
            Pose2d measurement,
            IsotropicNoiseSE2 stateSigma,
            IsotropicNoiseSE2 visionSigma) {
        // The difference between the odometry pose and the vision pose.

        Translation2d deltaTranslation = measurement.getTranslation().minus(sample.getTranslation());
        Rotation2d deltaRotation = measurement.getRotation().minus(sample.getRotation());

        double cartesianWeight = Uncertainty.cartesianWeight(stateSigma, visionSigma);
        double rotationWeight = Uncertainty.rotationWeight(stateSigma, visionSigma);

        // Scale the delta based on the noise.
        double deltaTranslationDistance = deltaTranslation.getNorm();
        Rotation2d deltaTranslationDirection = deltaTranslation.getAngle();

        double scaledTranslationDistance = deltaTranslationDistance * cartesianWeight;

        Translation2d scaledTranslation = new Translation2d(scaledTranslationDistance, deltaTranslationDirection);
        Rotation2d scaledRotation = deltaRotation.times(rotationWeight);

        Translation2d newTranslation = sample.getTranslation().plus(scaledTranslation);
        Rotation2d newRotation = sample.getRotation().plus(scaledRotation);

        Pose2d result = new Pose2d(newTranslation, newRotation);
        return result;
    }

}
