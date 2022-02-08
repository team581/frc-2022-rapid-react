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
import frc.robot.subsystems.CargoLimelightSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.InputFilter;

/** Aligns with the loading bay. */
public class LoadingBayAlignCommand extends SequentialCommandGroup {
  // TODO: Check if this goal Pose2d is correct - we should manually put the robot in the
  // desired position and then use those values as the goal pose
  private static final Pose2d GOAL = new Pose2d(0, Units.feetToMeters(1.5), new Rotation2d(0));
  private static final Pose2d TOLERANCE =
      new Pose2d(
          Units.inchesToMeters(1),
          Units.inchesToMeters(1),
          new Rotation2d(Units.degreesToRadians(5)));

  private final InputFilter inputFilter;

  public LoadingBayAlignCommand(
      DriveSubsystem drive, CargoLimelightSubsystem limelight, InputFilter inputFilter) {
    this.inputFilter = inputFilter;

    addCommands(
        new InstantCommand(() -> limelight.useVisionTarget(limelight.loadingBay)),
        new WaitForVisionTargetCommand(limelight),
        new AlignWithLimelightCommand(drive, limelight.loadingBay, GOAL, TOLERANCE));
  }

  @Override
  public void initialize() {
    inputFilter.useCargoControl();
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    inputFilter.useDriverControl();
  }
}
