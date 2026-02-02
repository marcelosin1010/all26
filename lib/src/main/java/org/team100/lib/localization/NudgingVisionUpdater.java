package org.team100.lib.localization;

import org.team100.lib.coherence.Takt;
import org.team100.lib.geometry.DeltaSE2;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
     * @param stateSigma  Standard deviation of the state.
     * @param visionSigma Standard deviation of the measurement.
     */
    @Override
    public void put(
            double timestamp,
            Pose2d measurement,
            IsotropicSigmaSE2 stateSigma,
            IsotropicSigmaSE2 visionSigma) {

        // Skip too-old measurement
        if (m_history.tooOld(timestamp)) {
            return;
        }

        // Sample the history at the measurement time.
        SwerveState sample = m_history.getRecord(timestamp);

        // Nudge the sample pose towards the measurement.
        Pose2d samplePose = sample.state().pose();
        Pose2d nudged = nudge(samplePose, measurement, stateSigma, visionSigma);

        // Use the interpolated gyro yaw, unmodified.
        Rotation2d gyroYaw = sample.gyroYaw();
        // Use the interpolated velocity, unmodified
        VelocitySE2 sampleVelocity = sample.state().velocity();

        ModelSE2 model = new ModelSE2(nudged, sampleVelocity);
        SwerveModulePositions positions = sample.positions();

        // Remember the result.
        m_history.put(timestamp, model, positions, gyroYaw);

        // Replay everything after the sample.
        m_odometryUpdater.replay(timestamp);

        // Remember the
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
     * Nudge the sample towards the measurement. This used to use a Twist, which
     * coupled the cartesian and rotational dimensions in an unphysical way; this
     * now treats the three dimensions as independent.
     * 
     * TODO: make these arrays into objects
     * 
     * @param sample      historical pose
     * @param measurement new input
     * @param stateSigma  standard deviation of the sample
     * @param visionSigma standard deviation of the measurement
     */
    static Pose2d nudge(
            Pose2d sample,
            Pose2d measurement,
            IsotropicSigmaSE2 stateSigma,
            IsotropicSigmaSE2 visionSigma) {
        // The difference between the odometry pose and the vision pose.
        DeltaSE2 delta = DeltaSE2.delta(sample, measurement);
        // Twist2d twist = sample.log(measurement);
        // Discount the twist based on the sigmas relative to each other.
        // Twist2d scaledTwist = Uncertainty.getScaledTwist(stateSigma, visionSigma,
        // twist);
        DeltaSE2 scaledDelta = Uncertainty.getScaledDelta(stateSigma, visionSigma, delta);
        return scaledDelta.plus(sample);
    }

}
