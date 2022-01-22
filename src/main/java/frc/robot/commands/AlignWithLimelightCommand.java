// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.vision.targets.LimelightVisionTarget;

/** Align with a vision target using the Limelight. */
public class AlignWithLimelightCommand extends CommandBase {
  private final LimelightVisionTarget visionTarget;
  private final DriveSubsystem driveSubsystem;
  private final Pose2d goal;
  private final Pose2d tolerance;

  /** Creates a new AlignWithLimelightCommand. */
  public AlignWithLimelightCommand(
      DriveSubsystem driveSubsystem,
      LimelightVisionTarget visionTarget,
      Pose2d goal,
      Pose2d tolerance) {
    addRequirements(driveSubsystem);

    this.driveSubsystem = driveSubsystem;
    this.visionTarget = visionTarget;
    this.goal = goal;
    this.tolerance = tolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.driveController.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: This will likely do bad things when the vision target is lost
    final var currentPose = visionTarget.alignment();
    final var chassisSpeeds =
        driveSubsystem.driveController.calculate(currentPose, goal, 0, goal.getRotation());

    driveSubsystem.driveWithSpeeds(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveSubsystem.driveController.atReference();
  }
}
