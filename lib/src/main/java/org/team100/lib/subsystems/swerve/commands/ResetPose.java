package org.team100.lib.subsystems.swerve.commands;

import org.team100.lib.localization.IsotropicNoiseSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Set the drivetrain pose to the specified pose.
 * 
 * This is useful to reset where "zero" is.
 * 
 * If you just want to reset rotation, use SetRotation.
 * 
 * WARNING: now that the robot uses tags more aggressively, this is useless and
 * maybe dangerous.
 */
public class ResetPose extends InstantCommand {

    public ResetPose(SwerveDriveSubsystem drive, Pose2d pose) {
        super(() -> {
            drive.resetPose(pose, IsotropicNoiseSE2.high());
            drive.stop();
        }, drive);
    }
}