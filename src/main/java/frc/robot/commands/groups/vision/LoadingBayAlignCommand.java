// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignWithLimelightCommand;
import frc.robot.commands.WaitForVisionTargetCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.InputFilter;

/** Aligns with the loading bay. */
public class LoadingBayAlignCommand extends SequentialCommandGroup {
  private static final Pose2d TOLERANCE =
      new Pose2d(
          Units.inchesToMeters(1),
          Units.inchesToMeters(1),
          new Rotation2d(Units.degreesToRadians(5)));

  public LoadingBayAlignCommand(
      DriveSubsystem drive, LimelightSubsystem limelight, InputFilter inputFilter) {
    addCommands(
        new InstantCommand(limelight.loadingBay::prepareForUse),
        new WaitForVisionTargetCommand(limelight),
        // TODO: Check if this goal Pose2d is correct - you should manually put the robot in the
        // desired position and then use those values as the goal pose
        new AlignWithLimelightCommand(
            drive,
            limelight.loadingBay,
            inputFilter,
            new Pose2d(0, Units.feetToMeters(1.5), new Rotation2d(0)),
            TOLERANCE));
  }
}
