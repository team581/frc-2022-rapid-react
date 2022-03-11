// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.paths.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.DriveSubsystem;
import frc.robot.vision.VisionTarget;
import java.util.function.Supplier;

/** Drives in a straight line from your current position to the desired position. */
public class BeelineCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Supplier<Pose2d> currentPoseSupplier;
  private final Pose2d goalPose;
  private final Supplier<Rotation2d> desiredRotationSupplier;

  /** Creates a new BeelineCommand. */
  public BeelineCommand(
      DriveSubsystem driveSubsystem,
      Supplier<Pose2d> currentPoseSupplier,
      Pose2d goalPose,
      Supplier<Rotation2d> desiredRotationSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.currentPoseSupplier = currentPoseSupplier;
    this.goalPose = goalPose;
    this.desiredRotationSupplier = desiredRotationSupplier;

    addRequirements(driveSubsystem);
  }

  /**
   * Creates a new BeelineCommand that uses the provided {@link VisionTarget vision target} for
   * getting the robot pose. The robot will be kept facing the vision target the entire time.
   */
  public BeelineCommand(DriveSubsystem driveSubsystem, VisionTarget visionTarget, Pose2d goalPose) {
    this(driveSubsystem, visionTarget::getRobotPose, goalPose, () -> new Rotation2d(0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setChassisSpeeds(
        driveSubsystem.driveController.calculate(
            currentPoseSupplier.get(), goalPose, 0, desiredRotationSupplier.get()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Don't do anything
    // Note that this will not stop your motors!
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveSubsystem.driveController.atReference();
  }
}
