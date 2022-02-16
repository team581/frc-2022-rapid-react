// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.util.AlignWithLimelightCommandFactory;
import frc.robot.subsystems.CargoLimelightSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.InputFilter;

/** Aligns with the loading bay. */
public class LoadingBayAlignCommand extends SequentialCommandGroup {
  // Because the vision target is (0, 0, 0) facing directly towards it means your rotation is 0,
  // which is why this can be a constant for the entire trajectory
  private static final Rotation2d GOAL_ROTATION = new Rotation2d(0);
  // TODO: Check if this goal Pose2d is correct - we should manually put the robot in the
  // desired position and then use those values as the goal pose
  private static final Pose2d GOAL = new Pose2d(0, 1, GOAL_ROTATION);

  private final InputFilter inputFilter;

  public LoadingBayAlignCommand(
      DriveSubsystem drive, CargoLimelightSubsystem limelight, InputFilter inputFilter) {
    this.inputFilter = inputFilter;
    final var commandFactory = new AlignWithLimelightCommandFactory(drive);

    addCommands(
        new InstantCommand(() -> limelight.useVisionTarget(limelight.loadingBay)),
        new WaitForVisionTargetCommand(limelight),
        // TODO: This "invert current angle" thing is untested
        commandFactory.generateCommand(limelight.loadingBay, GOAL, () -> limelight.loadingBay.getRobotPose().getRotation().unaryMinus()),
        new InstantCommand(drive::stopMotors));
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
