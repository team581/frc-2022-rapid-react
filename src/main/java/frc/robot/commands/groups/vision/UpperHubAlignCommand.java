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
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.WaitForVisionTargetCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.RumblePattern;
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

        // Wait for driver to point the Limelight at the upper hub while rumbling the controller
        race(
            new WaitForVisionTargetCommand(limelight),
            // Rumble the controller indefinitely, this is a parallel race so the other command will
            // interrupt this one
            new RumbleCommand(controller, new RumblePattern(1, 0.15, 0.15))),

        // TODO: Change to upper hub vision target
        // TODO: Check if this goal Pose2d is correct - these values need to be extracted once the
        // robot is manually aligned with the vision target
        new AlignWithLimelightCommand(
            driveSubsystem,
            vision.loadingBay,
            new Pose2d(0, Units.feetToMeters(1.5), new Rotation2d(0)),
            TOLERANCE),

        // Rumble until the command is interrupted by the driver
        new RumbleCommand(controller, new RumblePattern(1, 0.25, 0.15, 5)));
  }
}
