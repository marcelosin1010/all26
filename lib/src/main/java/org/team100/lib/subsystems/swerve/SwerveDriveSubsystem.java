package org.team100.lib.subsystems.swerve;

import java.util.List;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.DriverSkill;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.localization.FreshSwerveEstimate;
import org.team100.lib.localization.IsotropicNoiseSE2;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.EnumLogger;
import org.team100.lib.logging.LoggerFactory.VelocitySE2Logger;
import org.team100.lib.logging.LoggerFactory.ModelSE2Logger;
import org.team100.lib.music.Music;
import org.team100.lib.music.Player;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase implements VelocitySubsystemSE2, Music {
    // DEBUG produces a LOT of output, you should only enable it while you're
    // looking at it.
    private static final boolean DEBUG = false;
    private final FreshSwerveEstimate m_estimate;
    private final OdometryUpdater m_odometryUpdater;
    private final SwerveLocal m_swerveLocal;
    private final SwerveLimiter m_limiter;

    // CACHES
    private final ObjectCache<ModelSE2> m_stateCache;

    // LOGGERS
    private final ModelSE2Logger m_log_state;
    private final DoubleLogger m_log_turning;
    private final DoubleArrayLogger m_log_pose_array;
    private final EnumLogger m_log_skill;
    private final VelocitySE2Logger m_log_input;

    private final List<Player> m_players;

    public SwerveDriveSubsystem(
            LoggerFactory parent,
            OdometryUpdater odometryUpdater,
            FreshSwerveEstimate estimate,
            SwerveLocal swerveLocal,
            SwerveLimiter limiter) {
        LoggerFactory log = parent.type(this);
        m_estimate = estimate;
        m_odometryUpdater = odometryUpdater;
        m_swerveLocal = swerveLocal;
        m_limiter = limiter;
        m_stateCache = Cache.of(this::update);
        stop();
        m_log_state = log.modelSE2Logger(Level.COMP, "state");
        m_log_turning = log.doubleLogger(Level.TRACE, "Tur Deg");
        m_log_pose_array = log.doubleArrayLogger(Level.COMP, "pose array");
        m_log_skill = log.enumLogger(Level.TRACE, "skill level");
        m_log_input = log.VelocitySE2Logger(Level.TRACE, "drive input");
        m_players = m_swerveLocal.players();
    }

    ////////////////
    //
    // ACTUATORS
    //

    /**
     * Skip all scaling, limits generator, etc.
     * 
     * @param nextV for the next timestep
     */
    @Override
    public void setVelocity(VelocitySE2 nextV) {
        // keep the limiter up to date on what we're doing
        m_limiter.updateSetpoint(nextV);

        // Actuation is constant for the whole control period, which means
        // that to calculate robot-relative speed from field-relative speed,
        // we need to use the robot rotation *at the future time*.
        ModelSE2 currentState = getState();
        // Note this may add a bit of noise.
        ModelSE2 nextState = currentState.evolve(TimedRobot100.LOOP_PERIOD_S);
        Rotation2d nextTheta = nextState.rotation();

        ChassisSpeeds nextSpeed = SwerveKinodynamics.toInstantaneousChassisSpeeds(
                nextV, nextTheta);
        m_swerveLocal.setChassisSpeeds(nextSpeed);
        m_log_input.log(() -> nextV);
    }

    /**
     * Scales the supplied ChassisSpeed by the driver speed modifier.
     * 
     * @param speeds in robot coordinates
     */
    public void setChassisSpeeds(final ChassisSpeeds speeds) {
        // scale for driver skill; default is half speed.
        DriverSkill.Level driverSkillLevel = DriverSkill.level();
        m_swerveLocal.setChassisSpeeds(speeds.times(driverSkillLevel.scale()));
    }

    /**
     * Does not desaturate or optimize.
     * 
     * This "raw" mode is just for testing.
     */
    public void setRawModuleStates(SwerveModuleStates states) {
        m_swerveLocal.setRawModuleStates(states);
    }

    /** Use raw mode to set modules driving ahead */
    public Command aheadSlow() {
        return run(() -> setRawModuleStates(SwerveModuleStates.aheadSlow))
                .finallyDo(() -> stop());
    }

    /** Use robot-relative mode to set modules driving to the right */
    public Command rightwardSlow() {
        return run(() -> setChassisSpeeds(new ChassisSpeeds(0, -1.0, 0)))
                .finallyDo(() -> stop());
    }

    @Override
    public void stop() {
        m_swerveLocal.stop();
    }

    public void resetPose(Pose2d robotPose, IsotropicNoiseSE2 noise) {
        if (DEBUG)
            System.out.println("WARNING: Make sure resetting the swerve module collection doesn't break anything");
        m_swerveLocal.reset();
        m_odometryUpdater.reset(robotPose, noise);
        m_stateCache.reset();
    }

    ///////////////////////////////////////////////////////////////
    //
    // Observers
    //

    /**
     * Cached.
     * 
     * SwerveState representing the drivetrain's field-relative pose, velocity, and
     * acceleration.
     */
    @Override
    public ModelSE2 getState() {
        return m_stateCache.get();
    }

    ///////////////////////////////////////////////////////////////

    /**
     * Periodic() should not do actuation. Let commands do that.
     */
    @Override
    public void periodic() {
        if (DEBUG)
            System.out.println("drive periodic");
        // m_poseEstimator.periodic();
        // 4/2/25 Joel removed this state resetter because it happens earlier in
        // Robot.java
        // and i think we don't need to do it twice.
        // m_stateSupplier.reset();
        m_log_state.log(this::getState);
        m_log_turning.log(() -> getPose().getRotation().getDegrees());
        m_log_pose_array.log(this::poseArray);

        m_log_skill.log(() -> DriverSkill.level());
        m_swerveLocal.periodic();
    }

    private double[] poseArray() {
        return new double[] {
                getPose().getX(),
                getPose().getY(),
                getPose().getRotation().getDegrees()
        };
    }

    public void close() {
        m_swerveLocal.close();
    }

    /////////////////////////////////////////////////////////////////

    /**
     * Compute the current state. This is a fairly heavyweight thing to do, so it
     * should be cached (thus refreshed once per cycle).
     */
    private ModelSE2 update() {
        double now = Takt.get();
        SwerveModulePositions positions = m_swerveLocal.positions();
        // now that the pose estimator uses the SideEffect thing, we don't need this.
        // m_odometryUpdater.update();
        // m_cameraUpdater.run();
        ModelSE2 swerveModel = m_estimate.apply(now);
        if (DEBUG) {
            System.out.printf("update() positions %s estimated pose: %s\n", positions, swerveModel);
        }
        return swerveModel;
    }

    /** Return cached pose. */
    public Pose2d getPose() {
        return m_stateCache.get().pose();
    }

    /** Return cached velocity. */
    public VelocitySE2 getVelocity() {
        return m_stateCache.get().velocity();
    }

    /** Return cached speeds. */
    public ChassisSpeeds getChassisSpeeds() {
        return m_stateCache.get().chassisSpeeds();
    }

    @Override
    public Command play(double freq) {
        return run(() -> {
            m_swerveLocal.play(freq);
        });
    }

    @Override
    public List<Player> players() {
        return m_players;
    }

}
