package org.team100.frc2026.robot;

import java.io.IOException;

import org.team100.lib.coherence.Takt;
import org.team100.lib.indicator.Beeper;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.localization.NudgingVisionUpdater;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SimulatedTagDetector;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.GyroFactory;
import org.team100.lib.subsystems.swerve.SwerveDriveFactory;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.visualization.RobotPoseVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * This should contain all the hardware of the robot: all the subsystems etc
 * that the Binder and Auton classes may want to use.
 */
public class Machinery {
    // for background on drive current limits:
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    // https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
    // https://docs.google.com/document/d/10uXdmu62AFxyolmwtDY8_9UNnci7eVcev4Y64ZS0Aqk
    // https://github.com/frc1678/C2024-Public/blob/17e78272e65a6ce4f87c00a3514c79f787439ca1/src/main/java/com/team1678/frc2024/Constants.java#L195
    // 2/26/25: Joel updated the supply limit to 90A, see 1678 code above. This is
    // essentially unlimited, so you'll need to run some other kind of limiter (e.g.
    // acceleration) to keep from browning out.
    private static final double DRIVE_SUPPLY_LIMIT = 90;
    private static final double DRIVE_STATOR_LIMIT = 110;

    private static final LoggerFactory logger = Logging.instance().rootLogger;
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;

    private final Runnable m_robotViz;
    private final SwerveModuleCollection m_modules;
    private final Runnable m_simulatedTagDetector;

    final SwerveKinodynamics m_swerveKinodynamics;
    final AprilTagRobotLocalizer m_localizer;
    final SwerveDriveSubsystem m_drive;
    final Beeper m_beeper;

    public Machinery() {

        final LoggerFactory driveLog = logger.name("Drive");
        m_swerveKinodynamics = SwerveKinodynamicsFactory.get(driveLog);

        ////////////////////////////////////////////////////////////
        //
        // SUBSYSTEMS
        //

        // Subsystem initializers go here.

        ////////////////////////////////////////////////////////////
        //
        // VISUALIZATIONS
        //

        // Visualization initializers go here

        ////////////////////////////////////////////////////////////
        //
        // POSE ESTIMATION
        //
        m_modules = SwerveModuleCollection.get(
                driveLog,
                DRIVE_SUPPLY_LIMIT,
                DRIVE_STATOR_LIMIT,
                m_swerveKinodynamics);
        final Gyro gyro = GyroFactory.get(
                driveLog,
                m_swerveKinodynamics,
                m_modules);
        final SwerveHistory history = new SwerveHistory(
                driveLog,
                m_swerveKinodynamics,
                gyro.getYawNWU(),
                m_modules.positions(),
                Pose2d.kZero,
                Takt.get());
        final OdometryUpdater odometryUpdater = new OdometryUpdater(
                m_swerveKinodynamics, gyro, history, m_modules::positions);
        odometryUpdater.reset(Pose2d.kZero);
        final NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(
                history, odometryUpdater);

        ////////////////////////////////////////////////////////////
        //
        // CAMERA READERS
        //
        final AprilTagFieldLayoutWithCorrectOrientation layout = getLayout();

        m_localizer = new AprilTagRobotLocalizer(
                driveLog,
                fieldLogger,
                layout,
                history,
                visionUpdater);

        ////////////////////////////////////////////////////////////
        //
        // SIMULATED CAMERAS
        //
        m_simulatedTagDetector = SimulatedTagDetector.get(layout, history);

        ////////////////////////////////////////////////////////////
        //
        // DRIVETRAIN
        //
        m_drive = SwerveDriveFactory.get(
                driveLog,
                m_swerveKinodynamics,
                m_localizer,
                odometryUpdater,
                history,
                m_modules);
        m_drive.resetPose(new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d(Math.PI)));
        m_robotViz = new RobotPoseVisualization(
                fieldLogger, () -> m_drive.getState().pose(), "robot");

        ////////////////////////////////////////////////////////////
        //
        // INDICATOR
        //

        // There's no LED this year, unless we need it for testing or setup.

        // This makes beeps to warn about testing.
        m_beeper = new Beeper(m_drive);

    }

    public void periodic() {
        // publish the simulated tag sightings.
        m_simulatedTagDetector.run();
        m_robotViz.run();
    }

    public void close() {
        // this keeps the tests from conflicting via the use of simulated HAL ports.
        m_modules.close();
    }

    /** Trap the IO exception. */
    private static AprilTagFieldLayoutWithCorrectOrientation getLayout() {
        try {
            return new AprilTagFieldLayoutWithCorrectOrientation();
        } catch (IOException e) {
            throw new IllegalStateException("Could not read Apriltag layout file", e);
        }
    }
}
