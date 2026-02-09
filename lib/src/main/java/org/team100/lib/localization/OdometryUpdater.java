package org.team100.lib.localization;

import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Supplier;

import org.team100.lib.coherence.Takt;
import org.team100.lib.geometry.DeltaSE2;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Updates SwerveModelHistory with new odometry by selecting the most-recent
 * pose and applying the pose delta represented by the odometry, with the gyro
 * mixed in.
 * 
 * Note we use methods on the specific history implementation; the interface
 * won't work here.
 */
public class OdometryUpdater {
    private static final boolean DEBUG = false;

    private final SwerveKinodynamics m_kinodynamics;
    private final Gyro m_gyro;
    private final SwerveHistory m_history;
    private final Supplier<SwerveModulePositions> m_positions;

    public OdometryUpdater(
            SwerveKinodynamics kinodynamics,
            Gyro gyro,
            SwerveHistory estimator,
            Supplier<SwerveModulePositions> positions) {
        m_kinodynamics = kinodynamics;
        m_gyro = gyro;
        m_history = estimator;
        m_positions = positions;
    }

    /**
     * Put a new state estimate based on gyro and wheel data, from the suppliers
     * passed to the constructor. There is no history replay here, though it won't
     * fail if you give it out-of-order input.
     * 
     * It Samples the history before the specified time, and records a new pose
     * based on the difference in wheel positions between the sample and the
     * specified positions.
     * 
     * The gyro angle overrides the odometry-derived gyro measurement, and
     * the gyro rate overrides the rate derived from the difference to the previous
     * state.
     */
    public void update() {
        update(Takt.get());
    }

    /** For testing. */
    void update(double timestamp) {
        put(timestamp, m_gyro.getYawNWU(), m_positions.get());
    }

    /**
     * Empty the history and add the given measurements at the current instant.
     * 
     * Uses the module position supplier passed to the constructor, and the gyro.
     * When this is called by the bound command, it provides a pose with the current
     * translation and a rotation of zero (or 180 for the other button).
     */
    public void reset(Pose2d pose, IsotropicNoiseSE2 noise) {
        reset(pose, noise, Takt.get());
    }

    /** For testing. */
    public void reset(Pose2d pose, IsotropicNoiseSE2 noise, double timestampSeconds) {
        reset(m_gyro.getYawNWU(), pose, noise, timestampSeconds);
    }

    /**
     * Empty the history and add the given measurements.
     * 
     * Uses the module position supplier passed to the constructor.
     * When this is called by the bound command, it provides a pose with the current
     * translation and a rotation of zero (or 180 for the other button).
     * The gyro angle is whatever the gyro says, not zero.
     * 
     * Package private for testing.
     */
    void reset(
            Rotation2d gyroAngle,
            Pose2d pose,
            IsotropicNoiseSE2 noise,
            double timestampSeconds) {
        m_history.reset(
                m_positions.get(),
                pose,
                noise,
                timestampSeconds,
                gyroAngle);
    }

    ////////////////////////////////////////////////////

    private void put(
            double currentTimeS,
            Rotation2d gyroYaw,
            SwerveModulePositions positions) {

        // the entry right before this one, the basis for integration.
        Entry<Double, SwerveState> lowerEntry = m_history.lowerEntry(
                currentTimeS);

        if (lowerEntry == null) {
            // System.out.println("lower entry is null");
            // We're at the beginning. There's nothing to apply the wheel position delta to.
            // This should never happen.
            return;
        }

        double dt = currentTimeS - lowerEntry.getKey();
        SwerveState previousState = lowerEntry.getValue();

        ModelSE2 previousModel = previousState.state();
        IsotropicNoiseSE2 previousNoise = previousState.noise();

        if (DEBUG) {
            System.out.printf("previous x %.6f y %.6f\n",
                    previousModel.pose().getX(), previousModel.pose().getY());
        }

        SwerveModuleDeltas modulePositionDelta = SwerveModuleDeltas.modulePositionDelta(
                previousState.positions(), positions);
        if (DEBUG) {
            System.out.printf("modulePositionDelta %s\n", modulePositionDelta);
        }

        Twist2d twist = m_kinodynamics.getKinematics().toTwist2d(modulePositionDelta);
        if (DEBUG) {
            System.out.printf("twist x %.6f y %.6f theta %.6f\n", twist.dx, twist.dy, twist.dtheta);
        }

        double gyroDTheta = gyroYaw.minus(previousState.gyroYaw()).getRadians();
        twist = mix(twist, gyroDTheta, dt);

        // The new pose is just the twist applied to the old pose.
        Pose2d newPose = previousModel.pose().exp(twist);
        if (DEBUG) {
            System.out.printf("new pose x %.6f y %.6f\n", newPose.getX(), newPose.getY());
        }

        // Backward finite difference velocity.
        VelocitySE2 velocity = VelocitySE2.velocity(
                previousModel.pose(), newPose, dt);

        ModelSE2 model = new ModelSE2(newPose, velocity);

        // Noise in this update.
        IsotropicNoiseSE2 n0 = Uncertainty.odometryStdDevs(
                Metrics.translationalNorm(twist), twist.dtheta);

        // The variance of the sum of (independent) variables is
        // just the sum of their variances. So if you drive around for
        // awhile without seeing any tags, the variance will grow
        // without bound.
        IsotropicNoiseSE2 noise = previousNoise.plus(n0);

        m_history.put(currentTimeS, model, noise, positions, gyroYaw);
    }

    /**
     * Mix the gyro dtheta with the twist dtheta, depending on the twist norm.
     * If we're not moving very fast, then the odometry is pretty reliable: the gyro
     * signal is mostly drift. But if we're moving fast, then the wheels are
     * probably slipping against the carpet, so we should pay more attention to the
     * gyro signal. In particular, if we're not moving at all (or nearly so), we
     * should ignore the gyro entirely.
     */
    static Twist2d mix(Twist2d twist, double gyroDTheta, double dt) {
        double velocity = Metrics.l2Norm(twist) / dt;
        // Try a Gaussian. Set this to a very small number to recover the previous
        // behavior, where the gyro always overrides.
        double width = 1.0;
        double odoFraction = Math.exp(-width * velocity * velocity);
        double gyroFraction = 1 - odoFraction;
        return new Twist2d(
                twist.dx,
                twist.dy,
                odoFraction * twist.dtheta + gyroFraction * gyroDTheta);
    }

    /** Replay odometry after the sample time. */
    void replay(double sampleTime) {
        // Note the exclusive tailmap: we don't see the entry at timestamp.
        for (Map.Entry<Double, SwerveState> entry : m_history.exclusiveTailMap(sampleTime).entrySet()) {
            double timestamp = entry.getKey();
            SwerveState value = entry.getValue();
            Rotation2d gyroYaw = value.gyroYaw();
            SwerveModulePositions positions = value.positions();
            put(timestamp, gyroYaw, positions);
        }
    }

}
