package org.team100.frc2026.robot;

import java.util.function.BooleanSupplier;

import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.profile.se2.HolonomicProfileFactory;
import org.team100.lib.subsystems.se2.commands.DriveToPoseWithProfile;
import org.team100.lib.subsystems.swerve.commands.SetRotation;
import org.team100.lib.subsystems.swerve.commands.manual.DriveManually;
import org.team100.lib.subsystems.swerve.commands.manual.ManualChassisSpeeds;
import org.team100.lib.subsystems.swerve.commands.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.subsystems.swerve.commands.manual.ManualWithFullStateHeading;
import org.team100.lib.subsystems.swerve.commands.manual.ManualWithProfiledHeading;
import org.team100.lib.subsystems.swerve.commands.manual.ManualWithTargetLock;
import org.team100.lib.subsystems.swerve.commands.manual.SimpleManualModuleStates;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Binds buttons to commands. Also creates default commands.
 */
public class Binder {
    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;
    private final Machinery m_machinery;

    public Binder(Machinery machinery) {
        m_machinery = machinery;
    }

    public void bind() {
        final LoggerFactory log = rootLogger.name("Commands");

        /////////////////////////////////////////////////
        ///
        /// CONTROLS
        ///
        final DriverXboxControl driver = new DriverXboxControl(0);

        /////////////////////////////////////////////////
        //
        // DEFAULT COMMANDS
        //

        SwerveLimiter limiter = new SwerveLimiter(
                log,
                m_machinery.m_swerveKinodynamics,
                RobotController::getBatteryVoltage);
        limiter.updateSetpoint(m_machinery.m_drive.getVelocity());

        final DriveManually driveManually = new DriveManually(
                driver::velocity,
                m_machinery.m_localizer::setHeedRadiusM,
                m_machinery.m_drive,
                limiter);
        m_machinery.m_drive.setDefaultCommand(driveManually.withName("drive default"));
        final LoggerFactory manLog = log.type(driveManually);

        driveManually.register("MODULE_STATE", false,
                new SimpleManualModuleStates(manLog, m_machinery.m_swerveKinodynamics));

        driveManually.register("ROBOT_RELATIVE_CHASSIS_SPEED", false,
                new ManualChassisSpeeds(manLog, m_machinery.m_swerveKinodynamics));

        driveManually.register("FIELD_RELATIVE_TWIST", false,
                new ManualFieldRelativeSpeeds(manLog, m_machinery.m_swerveKinodynamics));

        final FeedbackR1 thetaFeedback = new PIDFeedback(
                log, 3.2, 0, 0, true, 0.05, 1);

        driveManually.register("SNAPS_PROFILED", true,
                new ManualWithProfiledHeading(
                        manLog,
                        m_machinery.m_swerveKinodynamics,
                        driver::pov,
                        thetaFeedback));

        driveManually.register("SNAPS_FULL_STATE", true,
                new ManualWithFullStateHeading(
                        manLog,
                        m_machinery.m_swerveKinodynamics,
                        driver::pov,
                        new double[] {
                                5,
                                0.35
                        }));

        /**
         * in reality, the target would come from some designator, e.g. buttons or
         * camera or whatever.
         */
        driveManually.register("LOCKED", false,
                new ManualWithTargetLock(
                        fieldLogger,
                        manLog,
                        m_machinery.m_swerveKinodynamics,
                        () -> new Translation2d(6, 4),
                        thetaFeedback));
        ///////////////////////////
        //
        // DRIVETRAIN
        //
        final LoggerFactory coralSequence = rootLogger.name("Coral Sequence");

        final ControllerSE2 holonomicController = ControllerFactorySE2.byIdentity(coralSequence);

        // Reset pose estimator so the current gyro rotation corresponds to zero.
        onTrue(driver::back,
                new SetRotation(m_machinery.m_drive, Rotation2d.kZero));
        onTrue(driver::a,
                new DriveToPoseWithProfile(
                fieldLogger, m_machinery.m_drive,holonomicController, HolonomicProfileFactory.get(
                    coralSequence, m_machinery.m_swerveKinodynamics, 1, 0.5, 1, 0.2), () -> new Pose2d(15.366,5.227, new Rotation2d(0)) 
                    ));

        // Reset pose estimator so the current gyro rotation corresponds to 180.
        onTrue(driver::start,
                new SetRotation(m_machinery.m_drive, Rotation2d.kPi));

        ////////////////////////////////////////////////////////////
        //
        // TEST
        //
        Tester tester = new Tester(m_machinery);
        whileTrue(() -> (RobotState.isTest() && driver.a() && driver.b()),
                tester.prematch());

    }

    private static Trigger whileTrue(BooleanSupplier condition, Command command) {
        return new Trigger(condition).whileTrue(command);
    }

    private static Trigger onTrue(BooleanSupplier condition, Command command) {
        return new Trigger(condition).onTrue(command);
    }
}
