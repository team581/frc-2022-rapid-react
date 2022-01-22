// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignWithLimelightCommand;
import frc.robot.commands.WaitForVisionTargetCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.vision.Vision;

/** Aligns with the upper hub. */
public class UpperHubAlignCommand extends SequentialCommandGroup {
  private static final Pose2d TOLERANCE =
      new Pose2d(
          Units.inchesToMeters(6),
          Units.inchesToMeters(6),
          new Rotation2d(Units.degreesToRadians(5)));

  public UpperHubAlignCommand(
      Vision vision,
      DriveSubsystem driveSubsystem,
      LimelightSubsystem limelight,
      XboxController controller) {
    addCommands(
        // Enable vision
        new InstantCommand(() -> vision.useVisionTarget(vision.upperHub)),
        new WaitForVisionTargetCommand(limelight),

        // TODO: Change to upper hub vision target
        // TODO: Check if this goal Pose2d is correct - these values need to be extracted once the
        // robot is manually aligned with the vision target
        new AlignWithLimelightCommand(
            driveSubsystem,
            vision.loadingBay,
            new Pose2d(0, Units.feetToMeters(1.5), new Rotation2d(0)),
            TOLERANCE));
  }
}
