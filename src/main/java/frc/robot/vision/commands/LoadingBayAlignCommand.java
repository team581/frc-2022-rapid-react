// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drive.DriveSubsystem;
import frc.robot.vision.commands.util.AlignWithVisionSystemCommandFactory;
import frc.robot.vision_cargo.CargoVisionSubsystem;

/** Aligns with the loading bay. */
public class LoadingBayAlignCommand extends SequentialCommandGroup {
  // Because the vision target is (0, 0, 0) facing directly towards it means your rotation is 0,
  // which is why this can be a constant for the entire trajectory
  private static final Rotation2d GOAL_ROTATION = new Rotation2d(0);
  // TODO: Check if this goal Pose2d is correct - we should manually put the robot in the desired
  // position and then use those values as the goal pose
  private static final Pose2d GOAL = new Pose2d(0, 1, GOAL_ROTATION);

  public LoadingBayAlignCommand(DriveSubsystem drive, CargoVisionSubsystem cargoVisionSubsystem) {
    final var commandFactory = new AlignWithVisionSystemCommandFactory(drive);

    addCommands(
        new UseVisionTargetCommand(cargoVisionSubsystem, cargoVisionSubsystem.loadingBay),
        // TODO: Refactor once pose estimation is implemented
        new WaitForVisionTargetCommand(cargoVisionSubsystem),
        // commandFactory.generateCommand(cargoVisionSubsystem.loadingBay, GOAL),
        // new BeelineCommand(drive, cargoVisionSubsystem.loadingBay, GOAL),
        new InstantCommand(drive::stopMotors));
  }
}
