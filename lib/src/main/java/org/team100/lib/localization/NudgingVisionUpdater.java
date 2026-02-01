package org.team100.lib.localization;

import org.team100.lib.coherence.Takt;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

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
    private double m_latestTimeS = 0;

    public NudgingVisionUpdater(
            SwerveHistory history,
            OdometryUpdater odometryUpdater) {
        m_history = history;
        m_odometryUpdater = odometryUpdater;
    }

    /**
     * Put a new state estimate based on the supplied pose. If not current,
     * subsequent wheel updates are replayed.
     */
    @Override
    public void put(
            double timestampS,
            Pose2d measurement,
            double[] stateSigma,
            double[] visionSigma) {

        // Skip too-old measurement
        if (m_history.tooOld(timestampS)) {
            return;
        }

        // Sample the history at the measurement time.
        SwerveState sample = m_history.getRecord(timestampS);

        // If there is a sample, nudge it towards the measurement.
        Pose2d nudged = nudge(
                sample.state().pose(), measurement, stateSigma, visionSigma);
        m_history.put(
                timestampS,
                new ModelSE2(nudged, sample.state().velocity()),
                sample.positions());
        m_odometryUpdater.replay(timestampS);
        m_latestTimeS = Takt.get();
    }

    /**
     * The age of the last pose estimate, in seconds.
     * The caller could use this to, say, indicate tag visibility.
     */
    public double getPoseAgeSec() {
        double now = Takt.get();
        return now - m_latestTimeS;
    }

    /////////////////////////////////////////

    /**
     * Nudge the sample towards the measurement.
     */
    static Pose2d nudge(
            Pose2d sample,
            Pose2d measurement,
            double[] stateSigma,
            double[] visionSigma) {
        // Step 2: Measure the twist between the odometry pose and the vision pose.
        Twist2d twist = sample.log(measurement);
        // Step 4: Discount the twist based on the sigmas relative to each other
        Twist2d scaledTwist = Uncertainty.getScaledTwist(stateSigma, visionSigma, twist);
        return sample.exp(scaledTwist);
    }

}
