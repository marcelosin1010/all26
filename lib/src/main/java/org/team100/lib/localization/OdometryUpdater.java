package org.team100.lib.localization;

import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Supplier;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.fusion.CovarianceInflation;
import org.team100.lib.fusion.Fusor;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.SwerveStateLogger;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.Uncertainty;
import org.team100.lib.uncertainty.VariableR1;
import org.team100.lib.util.StrUtil;

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
    private final Fusor m_gyroBiasFusor;
    private final Fusor m_rotationFusor;

    private final SwerveStateLogger m_logState;

    public OdometryUpdater(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            Gyro gyro,
            SwerveHistory estimator,
            Supplier<SwerveModulePositions> positions) {
        LoggerFactory log = parent.type(this);
        m_kinodynamics = kinodynamics;
        m_gyro = gyro;
        m_history = estimator;
        m_positions = positions;
        double dt = TimedRobot100.LOOP_PERIOD_S;
        // bias scales with dt^1.5.
        m_gyroBiasFusor = new CovarianceInflation(0.02, gyro.bias_noise() * dt * Math.sqrt(dt));
        m_rotationFusor = new CovarianceInflation(0.02, 0.003);
        m_logState = log.swerveStateLogger(Level.TRACE, "state");
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
        SwerveState newState = update(Takt.get());
        if (newState != null)
            m_logState.log(() -> newState);
    }

    /** For testing. */
    SwerveState update(double timestamp) {
        return put(timestamp, m_gyro.getYawNWU(), m_positions.get());
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

    /**
     * Empty the history and add the given measurements.
     * 
     * Uses the module position supplier passed to the constructor.
     * When this is called by the bound command, it provides a pose with the current
     * translation and a rotation of zero (or 180 for the other button).
     * The gyro angle is whatever the gyro says, not zero.
     * 
     * New! Adds a very uncertain gyro bias estimate.
     */
    public void reset(
            Pose2d pose,
            IsotropicNoiseSE2 noise,
            double timestampSeconds) {
        // No idea what the gyro bias is.
        VariableR1 gyroBias = VariableR1.fromVariance(0, 1);
        m_history.reset(
                m_positions.get(),
                pose,
                noise,
                timestampSeconds,
                m_gyro.getYawNWU(),
                gyroBias);
    }

    ////////////////////////////////////////////////////

    /**
     * Add a SwerveState to the buffer at the specified time, based on the measured
     * yaw and positions.
     * 
     * @param currentTimeS takt time, seconds
     * @param gyroYaw      verbatim gyro measurement
     * @param positions    verbatim drive measurement
     */
    private SwerveState put(
            double currentTimeS,
            Rotation2d gyroYaw,
            SwerveModulePositions positions) {
        // System.out.printf("gyroYaw %s\n", gyroYaw);

        // the entry right before this one, the basis for integration.
        Entry<Double, SwerveState> lowerEntry = m_history.lowerEntry(
                currentTimeS);

        if (lowerEntry == null) {
            // System.out.println("lower entry is null");
            // We're at the beginning. There's nothing to apply the wheel position delta to.
            // This should never happen.
            return null;
        }

        double dt = currentTimeS - lowerEntry.getKey();
        SwerveState previousState = lowerEntry.getValue();

        SwerveState newState = newState(previousState, dt, gyroYaw, positions);

        m_history.put(currentTimeS, newState);
        return newState;
    }

    /**
     * Compute the new state, based on the previous state.
     */
    SwerveState newState(
            SwerveState previousState,
            double dt,
            Rotation2d gyroYaw,
            SwerveModulePositions positions) {

        ModelSE2 previousModel = previousState.state();

        Pose2d previousPose = previousModel.pose();
        if (DEBUG) {
            System.out.printf("previous x %.6f y %.6f\n",
                    previousPose.getX(), previousPose.getY());
        }

        IsotropicNoiseSE2 previousNoise = previousState.noise();
        SwerveModulePositions previousPositions = previousState.positions();
        Rotation2d previousGyroYaw = previousState.gyroYaw();
        VariableR1 previousGyroBias = previousState.gyroBias();

        SwerveModuleDeltas modulePositionDelta = SwerveModuleDeltas.modulePositionDelta(
                previousPositions, positions);
        if (DEBUG) {
            System.out.printf("modulePositionDelta %s\n", modulePositionDelta);
        }

        Twist2d twist = m_kinodynamics.getKinematics().toTwist2d(modulePositionDelta);
        if (DEBUG) {
            System.out.printf("twist %s\n", StrUtil.twistStr(twist));
        }

        // Gyro increment in this step
        double gyroStep = gyroYaw.minus(previousGyroYaw).getRadians();

        // Noise in the increment is the noise density times the sample time.
        double gyroWhiteNoise = m_gyro.white_noise() * Math.sqrt(dt);
        VariableR1 gyroMeasurement = VariableR1.fromStdDev(gyroStep, gyroWhiteNoise);

        // Cartesian distance in this step
        double distanceM = Metrics.translationalNorm(twist);
        // Rotation in this step
        double rotationRad = twist.dtheta;

        double odoDTheta = twist.dtheta;
        double odoDThetaStdDev = Uncertainty.odometryRotationStdDev(
                distanceM, rotationRad);

        VariableR1 odoRotationMeasurement = VariableR1.fromStdDev(
                odoDTheta, odoDThetaStdDev);

        // Gyro drift during this time step
        // Variance here is the (constant) gyro noise and the (speed-dependent) odo
        // noise.
        VariableR1 gyroBiasMeasurement = VariableR1.subtract(
                gyroMeasurement, odoRotationMeasurement);

        // gyro bias has a very low minimum variance
        VariableR1 newGyroBiasEstimate = m_gyroBiasFusor.fuse(
                previousGyroBias, gyroBiasMeasurement);

        // Gyro increment without the drift.
        VariableR1 correctedGyroMeasurement = VariableR1.subtract(
                gyroMeasurement, newGyroBiasEstimate);

        // Fuse the odometry and gyro rotations
        VariableR1 fusedRotationMeasurement = m_rotationFusor.fuse(
                odoRotationMeasurement, correctedGyroMeasurement);

        // use the fused rotation as the step estimate
        twist = new Twist2d(twist.dx, twist.dy, fusedRotationMeasurement.mean());

        // The new pose is just the twist applied to the old pose.
        Pose2d newPose = previousPose.exp(twist);

        // Compute a new velocity using backward finite difference.
        VelocitySE2 velocity = VelocitySE2.velocity(previousPose, newPose, dt);

        ModelSE2 model = new ModelSE2(newPose, velocity);

        // Noise in this update.
        double cartesianNoise = Uncertainty.odometryCartesianStdDev(distanceM);
        IsotropicNoiseSE2 n0 = IsotropicNoiseSE2.fromStdDev(
                cartesianNoise, fusedRotationMeasurement.sigma());

        // The variance of the sum of (independent) variables is
        // just the sum of their variances. So if you drive around for
        // awhile without seeing any tags, the variance will grow
        // without bound.
        IsotropicNoiseSE2 noise = previousNoise.plus(n0);

        // gyro and position measurements are verbatim
        SwerveState swerveState = new SwerveState(
                model,
                noise,
                positions,
                gyroYaw,
                newGyroBiasEstimate);
        return swerveState;
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
            // System.out.printf("==== REPLAY %f\n", timestamp);
            SwerveState value = entry.getValue();
            Rotation2d gyroYaw = value.gyroYaw();
            SwerveModulePositions positions = value.positions();
            put(timestamp, gyroYaw, positions);
        }
    }

}
