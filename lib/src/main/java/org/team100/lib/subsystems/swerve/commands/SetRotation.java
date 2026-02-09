package org.team100.lib.subsystems.swerve.commands;

import org.team100.lib.localization.IsotropicNoiseSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Set the rotation of the robot pose to the specified rotation.
 * 
 * This is good for resetting the "zero," in cases where the robot isn't
 * actually facing zero (e.g. 180)
 * 
 * WARNING: now that the robot uses tag rotation, this command is useless and
 * maybe dangerous.
 */
public class SetRotation extends InstantCommand {

    public SetRotation(
            SwerveDriveSubsystem drive,
            Rotation2d rotation) {
        super(() -> drive.resetPose(
                new Pose2d(drive.getPose().getTranslation(), rotation),
                IsotropicNoiseSE2.high()),
                drive);
    }
}
