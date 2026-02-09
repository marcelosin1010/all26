package org.team100.lib.localization;

import java.util.Map.Entry;
import java.util.SortedMap;
import java.util.function.DoubleFunction;

import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.util.TimeInterpolatableBuffer100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * History is just a container, in fact the implementation is little more than a
 * wrapper around TimeInterpolatableBuffer.
 * 
 * The history always has *something* in it, even the initial zero pose.
 * 
 * There are no dependencies managed here; for that, use SwerveModelEstimate.
 * 
 * Note this should only be used from within the localization package.
 * 
 * Other SwerveModel consumers should use SwerveModelEstimate.
 */
public class SwerveHistory implements DoubleFunction<ModelSE2> {
    /**
     * The buffer only needs to be long enough to catch stale-but-still-helpful
     * vision updates.
     * 
     * The current Raspberry Pi cameras seem to be able to provide frames to RoboRIO
     * code with about 75-100 ms latency. There will never be a vision update
     * older than about 200 ms.
     */
    private static final double BUFFER_DURATION = 0.2;

    private final DoubleLogger m_log_timestamp;
    private final SwerveKinodynamics m_kinodynamics;
    private final TimeInterpolatableBuffer100<SwerveState> m_poseBuffer;

    public SwerveHistory(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            Rotation2d gyroAngle,
            SwerveModulePositions modulePositions,
            Pose2d initialPoseMeters,
            IsotropicNoiseSE2 noise,
            double timestampSeconds) {
        m_log_timestamp = parent.type(this).doubleLogger(Level.TRACE, "sample timestamp");
        m_kinodynamics = kinodynamics;
        ModelSE2 state = new ModelSE2(initialPoseMeters, new VelocitySE2(0, 0, 0));
        SwerveState initialState = new SwerveState(
                m_kinodynamics.getKinematics(),
                state,
                noise,
                modulePositions,
                gyroAngle);
        m_poseBuffer = new TimeInterpolatableBuffer100<>(
                BUFFER_DURATION, timestampSeconds, initialState);
    }

    /**
     * Sample the state estimate buffer.
     */
    @Override
    public ModelSE2 apply(double timestampSeconds) {
        m_log_timestamp.log(() -> timestampSeconds);
        return m_poseBuffer.get(timestampSeconds).state();
    }

    /** Empty the buffer and add the given measurements. */
    void reset(
            SwerveModulePositions modulePositions,
            Pose2d pose,
            IsotropicNoiseSE2 noise,
            double timestampSeconds,
            Rotation2d gyroYaw) {
        // empty the buffer and add the current pose
        ModelSE2 model = new ModelSE2(pose, new VelocitySE2(0, 0, 0));
        SwerveState state = new SwerveState(
                m_kinodynamics.getKinematics(),
                model,
                noise,
                modulePositions,
                gyroYaw);
        m_poseBuffer.reset(timestampSeconds, state);
    }

    //////////////////////////////////////////////////
    // methods below are for history maintenance

    /**
     * timestamp in seconds
     */
    void put(
            double timestamp,
            ModelSE2 model,
            IsotropicNoiseSE2 noise,
            SwerveModulePositions positions,
            Rotation2d gyroYaw) {
        m_poseBuffer.put(
                timestamp,
                new SwerveState(
                        m_kinodynamics.getKinematics(),
                        model,
                        noise,
                        positions,
                        gyroYaw));
    }

    Entry<Double, SwerveState> lowerEntry(double timestamp) {
        return m_poseBuffer.lowerEntry(timestamp);
    }

    SwerveState getRecord(double timestamp) {
        return m_poseBuffer.get(timestamp);
    }

    boolean tooOld(double timestamp) {
        return m_poseBuffer.tooOld(timestamp);
    }

    SortedMap<Double, SwerveState> exclusiveTailMap(double timestamp) {
        return m_poseBuffer.tailMap(timestamp, false);
    }

    int size() {
        return m_poseBuffer.size();
    }

    double lastKey() {
        return m_poseBuffer.lastKey();
    }

}
