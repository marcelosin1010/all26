package org.team100.lib.localization;

import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.stream.DoubleStream;

import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.EnumLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.logging.LoggerFactory.Transform3dLogger;
import org.team100.lib.network.CameraReader;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.util.TrailingHistory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Extracts robot pose estimates from camera observations of AprilTags.
 * 
 * Note this class depends only on the state *history*, not on the coherent sate
 * *estimate*. The camera input doesn't require fresh odometry, it modifies the
 * past (and replays up to the present).
 */
public class AprilTagRobotLocalizer extends CameraReader<Blip24> {
    private static final boolean DEBUG = false;

    /** Maximum age of the sights we publish for diagnosis. */
    private static final double HISTORY_DURATION = 1.0;

    /**
     * If the tag is closer than this threshold, then the camera's estimate of tag
     * rotation might be more accurate than the gyro, so we use the camera's
     * estimate of tag rotation to update the robot pose. If the tag is further away
     * than this, then the camera-derived rotation is probably less accurate than
     * the gyro, so we use the gyro instead.
     * 
     * Set this to zero to disable tag-derived rotation and always use the gyro.
     * 
     * Set this to some large number (e.g. 100) to disable gyro-derived rotation and
     * always use the camera.
     */
    private static final double TAG_ROTATION_BELIEF_THRESHOLD_M = 100;

    /** Discard results further than this from the previous one. */
    private static final double VISION_CHANGE_TOLERANCE_M = 0.1;
    // private static final double VISION_CHANGE_TOLERANCE_M = 1;

    private final DoubleFunction<ModelSE2> m_history;
    private final VisionUpdater m_visionUpdater;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;

    /**
     * The apparent position of tags we see: this can be shown in AdvantageScope
     * using the Vision Target feature. The apparent position should match the
     * actual position, if the cameras are calibrated correctly. Note this involves
     * matching the frame timestamp with the pose history timestamp, so if the blip
     * source timestamp is wrong (as it is at the moment in the simulated tag
     * detector) then these positions will be a little bit wrong.
     */
    private final StructArrayPublisher<Pose3d> m_pub_tags;
    /** Just tags we use for pose estimation. */
    private final StructArrayPublisher<Pose3d> m_pub_used_tags;
    /** Logging the same thing for the Glass Field2d widget, for simulation. */
    private final DoubleArrayLogger m_log_allTags;
    private final DoubleArrayLogger m_log_usedTags;

    /**
     * The pose we derive from each sighting, so we can see it in AdvantageScope's
     * map, which can't understand our usual Pose2dLogger's output.
     */
    private final StructPublisher<Pose2d> m_pub_pose;

    // LOGGERS
    private final EnumLogger m_log_alliance;
    private final DoubleLogger m_log_heedRadius;
    private final BooleanLogger m_log_using_gyro;
    private final DoubleLogger m_log_tag_error;
    private final Pose2dLogger m_log_pose;
    /** For calibration. */
    private final Transform3dLogger m_log_tag_in_camera;

    /**
     * The difference between the current instant and the instant of the blip,
     * including our magic correction, i.e. this is the time we look up in the pose
     * buffer.
     */
    private final DoubleLogger m_log_lag;

    // Remember the previous vision-based pose estimate, so we can measure the
    // distance between consecutive updates, and ignore too-far updates.
    private Pose2d m_prevPose;

    /** use tags closer than this; ignore tags further than this. */
    private double m_heedRadiusM = 3.5;

    /**
     * Accumulates all tags we receive in each cycle, whether we use them or not.
     */
    private final TrailingHistory<Pose3d> m_allTags;
    /**
     * Just the tags we use for pose estimation, i.e. not ones that are too far
     * away.
     */
    private final TrailingHistory<Pose3d> m_usedTags;

    /**
     * @param parent        logger
     * @param layout        map of apriltags
     * @param history       f(timestamp) = swerve state, use SwerveModelHistory.
     * @param visionUpdater mutates history
     */
    public AprilTagRobotLocalizer(
            LoggerFactory parent,
            LoggerFactory fieldLogger,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            DoubleFunction<ModelSE2> history,
            VisionUpdater visionUpdater) {
        super(parent, "vision", "blips", StructBuffer.create(Blip24.struct));
        LoggerFactory log = parent.type(this);
        m_layout = layout;
        m_history = history;
        m_visionUpdater = visionUpdater;
        m_allTags = new TrailingHistory<>(HISTORY_DURATION);
        m_usedTags = new TrailingHistory<>(HISTORY_DURATION);

        m_log_allTags = fieldLogger.doubleArrayLogger(Level.TRACE, "all tags");
        m_log_usedTags = fieldLogger.doubleArrayLogger(Level.TRACE, "used tags");

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_pub_tags = inst.getStructArrayTopic("tags", Pose3d.struct).publish();
        m_pub_used_tags = inst.getStructArrayTopic("used tags", Pose3d.struct).publish();
        m_pub_pose = inst.getStructTopic("pose", Pose2d.struct).publish();

        m_log_alliance = log.enumLogger(Level.TRACE, "alliance");
        m_log_heedRadius = log.doubleLogger(Level.TRACE, "heed radius");
        m_log_using_gyro = log.booleanLogger(Level.TRACE, "rotation source");
        m_log_tag_error = log.doubleLogger(Level.TRACE, "tag error");
        m_log_pose = log.pose2dLogger(Level.TRACE, "pose");
        m_log_tag_in_camera = log.transform3dLogger(Level.TRACE, "tag in camera");
        m_log_lag = log.doubleLogger(Level.TRACE, "lag");
    }

    @Override
    protected void perValue(
            Transform3d cameraOffset,
            double valueTimestamp,
            Blip24[] blips) {
        estimateRobotPose(
                cameraOffset,
                blips,
                valueTimestamp,
                DriverStation.getAlliance());
    }

    @Override
    protected void finishUpdate() {
        m_pub_tags.set(m_allTags.getAll().toArray(new Pose3d[0]));
        m_pub_used_tags.set(m_usedTags.getAll().toArray(new Pose3d[0]));
        m_log_allTags.log(
                () -> m_allTags.getAll().stream().flatMapToDouble(
                        x -> DoubleStream.of(x.getX(), x.getY(), x.toPose2d().getRotation().getDegrees())).toArray());
        m_log_usedTags.log(
                () -> m_usedTags.getAll().stream().flatMapToDouble(
                        x -> DoubleStream.of(x.getX(), x.getY(), x.toPose2d().getRotation().getDegrees())).toArray());
    }

    /**
     * Tags outside this radius are ignored.
     */
    public void setHeedRadiusM(double heedRadiusM) {
        m_heedRadiusM = heedRadiusM;
    }

    /**
     * Compute the robot pose and put it in the pose estimator.
     * 
     * @param cameraOffset   Camera pose in robot coordinates
     * @param blips          The targets in the current camera frame
     * @param valueTimestamp Camera frame timestamp
     * @param optAlliance    From the driver station, it's here to make testing
     *                       easier.
     */
    void estimateRobotPose(
            Transform3d cameraOffset,
            Blip24[] blips,
            double valueTimestamp,
            Optional<Alliance> optAlliance) {

        // Vasili added this extra delay after some experimentation, but
        // it breaks simulation, so I set it back to zero.
        // The effect is to make the received sight
        // appear as if it were from further in the past than the timestamp says it is,
        // which would be required if there were delay (a lot of delay) not included
        // in the timestamp.
        // TODO: figure out what this does and describe it describe here.
        // final double IMPORTANT_MAGIC_NUMBER = 0.027;
        final double IMPORTANT_MAGIC_NUMBER = 0.0;
        double correctedTimestamp = valueTimestamp - IMPORTANT_MAGIC_NUMBER;

        // this seems to always be 1. ????
        // TODO: look more closely at this
        m_log_lag.log(() -> Takt.get() - correctedTimestamp);

        if (!optAlliance.isPresent()) {
            // this happens on startup
            if (DEBUG)
                System.out.println("WARNING: VisionDataProvider24: Alliance is not present!");
            return;
        }
        Alliance alliance = optAlliance.get();
        m_log_alliance.log(() -> alliance);
        m_log_heedRadius.log(() -> m_heedRadiusM);

        Pose2d historicalPose = historicalPose(correctedTimestamp);

        for (int i = 0; i < blips.length; ++i) {
            Blip24 blip = blips[i];

            printBlip(blip);

            // Look up the pose of the tag in the field frame.
            Optional<Pose3d> tagInFieldCoordsOptional = m_layout.getTagPose(alliance, blip.getId());
            if (!tagInFieldCoordsOptional.isPresent()) {
                // This shouldn't happen, but it does.
                System.out.printf("WARNING: VisionDataProvider24: no tag for id %d\n", blip.getId());
                continue;
            }

            // Field-to-tag, canonical pose from the JSON file
            Pose3d tagInField = tagInFieldCoordsOptional.get();

            Transform3d tagInCamera = tagInCamera(blip);

            printForCalibration(cameraOffset, blip, tagInCamera);

            tagInCamera = maybeOverrideRotation(cameraOffset, historicalPose, tagInField, tagInCamera);

            Pose3d estimatedTagInField = estimatedTagInField(cameraOffset, historicalPose, tagInCamera);

            m_allTags.add(correctedTimestamp, estimatedTagInField);

            logTagError(tagInField, estimatedTagInField);

            Pose2d robotPose2d = robotPose2d(historicalPose, cameraOffset, tagInField, tagInCamera);

            // Clean the used-tags collection in case we don't end up writing to it.
            m_usedTags.cleanup(correctedTimestamp);

            //////////////////////////////////////////////////////////////////
            ///
            /// Should we use this update?
            ///
            if (!Experiments.instance.enabled(Experiment.HeedVision)) {
                // No, we've turned vision off.
                continue;
            }
            ///
            if (tagInCamera.getTranslation().getNorm() > m_heedRadiusM) {
                // No, the tag is too far away.
                continue;
            }
            ///
            if (m_prevPose == null) {
                // No, we need another nearby fix to believe either one.
                m_prevPose = robotPose2d;
                continue;
            }
            ///
            double distanceM = Metrics.translationalDistance(m_prevPose, robotPose2d);
            if (distanceM > VISION_CHANGE_TOLERANCE_M) {
                // No, the new estimate is too far from the previous one.
                m_prevPose = robotPose2d;
                continue;
            }
            ///
            /// Yes, we should use this update.
            ///
            //////////////////////////////////////////////////////////////////

            m_usedTags.add(correctedTimestamp, estimatedTagInField);
            m_visionUpdater.put(
                    correctedTimestamp,
                    robotPose2d,
                    Uncertainty.stateStdDevs(),
                    Uncertainty.visionMeasurementStdDevs(distanceM));
            m_prevPose = robotPose2d;
        }
    }

    /**
     * Project the 3d pose into a Pose2d, but use the historical pose rotation
     * (which is the gyro rotation from that timestamp)
     */
    private Pose2d robotPose2d(
            Pose2d historicalPose,
            Transform3d cameraInRobot,
            Pose3d tagInField,
            Transform3d tagInCamera) {
        Pose3d robotPose3d = PoseEstimationHelper.robotInField(
                cameraInRobot, tagInField, tagInCamera);
        Pose2d robotPose2d = new Pose2d(
                robotPose3d.getTranslation().toTranslation2d(),
                historicalPose.getRotation());
        m_log_pose.log(() -> robotPose2d);
        m_pub_pose.set(robotPose2d);
        return robotPose2d;
    }

    /**
     * If the tag is too far, replace the blip-derived tag rotation with a
     * gyro-derived tag rotation.
     */
    private Transform3d maybeOverrideRotation(
            Transform3d cameraOffset, Pose2d historicalPose, Pose3d tagInField, Transform3d tagInCamera) {
        if (tagInCamera.getTranslation().getNorm() > TAG_ROTATION_BELIEF_THRESHOLD_M) {
            m_log_using_gyro.log(() -> true);
            tagInCamera = PoseEstimationHelper.tagInCamera(
                    cameraOffset, tagInField, tagInCamera, new Rotation3d(historicalPose.getRotation()));
        } else {
            m_log_using_gyro.log(() -> false);
        }
        return tagInCamera;
    }

    /** Log the norm of the translational error of the tag. */
    private void logTagError(Pose3d tagInField, Pose3d estimatedTagInField) {
        Transform3d tagError = tagInField.minus(estimatedTagInField);
        m_log_tag_error.log(() -> tagError.getTranslation().getNorm());
    }

    /**
     * Returns the pose from the frame timestamp.
     * 
     * Note this pulls from the *old history*, not the *odometry-updated history*,
     * because we don't care about the latest odometry update.
     * 
     * Because the camera delay is much more than the odometry delay, we're always
     * trying to write history from several cycles ago (followed by replay). It's ok
     * for new odometry to be the last thing.
     */
    private Pose2d historicalPose(double correctedTimestamp) {
        Pose2d historicalPose = m_history.apply(correctedTimestamp).pose();
        Rotation2d gyroRotation = historicalPose.getRotation();
        if (DEBUG) {
            System.out.printf("historical gyro rotation %f\n", gyroRotation.getRadians());
        }
        return historicalPose;
    }

    private void printBlip(Blip24 blip) {
        if (!DEBUG)
            return;
        Translation3d t = blip.getRawPose().getTranslation();
        Rotation3d r = blip.getRawPose().getRotation();
        System.out.printf("blip raw pose %d X %5.2f Y %5.2f Z %5.2f R %5.2f P %5.2f Y %5.2f\n",
                blip.getId(), t.getX(), t.getY(), t.getZ(), r.getX(), r.getY(), r.getZ());
    }

    /**
     * This is used for camera offset calibration. Place a tag at a known position,
     * observe the offset, and add it to Camera.java, inverted.
     */
    private void printForCalibration(Transform3d cameraOffset, Blip24 blip, Transform3d tagInCamera) {
        if (!DEBUG)
            return;
        Transform3d tagInRobot = cameraOffset.plus(tagInCamera);
        System.out.printf("tagInRobot id %d X %5.2f Y %5.2f Z %5.2f R %5.2f P %5.2f Y %5.2f\n",
                blip.getId(), tagInRobot.getTranslation().getX(), tagInRobot.getTranslation().getY(),
                tagInRobot.getTranslation().getZ(), tagInRobot.getRotation().getX(),
                tagInRobot.getRotation().getY(), tagInRobot.getRotation().getZ());
    }

    /** The estimated tag position, for visualization of vision error. */
    private Pose3d estimatedTagInField(
            Transform3d cameraOffset, Pose2d historicalPose, Transform3d tagInCamera) {
        // Field-to-robot
        Pose3d historicalPose3d = new Pose3d(historicalPose);
        // Field-to-robot plus robot-to-camera = field-to-camera
        Pose3d historicalCameraInField = historicalPose3d.transformBy(cameraOffset);
        // Given the historical pose, where do we think the tag is?
        return historicalCameraInField.transformBy(tagInCamera);
    }

    /** Camera-to-tag, as it appears in the camera frame. */
    private Transform3d tagInCamera(Blip24 blip) {
        Transform3d blipTransform = blip.blipToTransform();
        m_log_tag_in_camera.log(() -> blipTransform);
        return blipTransform;
    }

}