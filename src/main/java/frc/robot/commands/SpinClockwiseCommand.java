// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class SpinClockwiseCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final GyroSubsystem gyroSubsystem;

  private Rotation2d initialAngle;

  /** Creates a new SpinInPlaceCommand. */
  public SpinClockwiseCommand(DriveSubsystem driveSubsystem, GyroSubsystem gyroSubsystem) {
    addRequirements(driveSubsystem, gyroSubsystem);

    this.driveSubsystem = driveSubsystem;
    this.gyroSubsystem = gyroSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = Rotation2d.fromDegrees(gyroSubsystem.gyro.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveSubsystem.driveController.atReference();
  }

  private ChassisSpeeds getDriveSpeeds() {
    final var currentPose = Rotation2d.fromDegrees(gyroSubsystem.gyro.getAngle());
    final var poseRef = initialAngle.plus(Rotation2d.fromDegrees(360 - 1));

    // TODO: Fix this, perhaps just try driving forward insteadd of rotating
    // return driveSubsystem.driveController.calculate(new Pose2d(0, 0, currentPose), new Pose2d(0,
    // 0, currentPose), poseRef);

    return null;
  }
}
