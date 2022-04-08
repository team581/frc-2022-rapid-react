// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controller.DriveController;
import frc.robot.drive.DriveSubsystem;

public class TeleopDriveCommand extends CommandBase {
  /** The maximum allowed turn rate of the robot during teleop, per second. */
  public static final Rotation2d MAX_TELEOP_TURN_RATE = Rotation2d.fromDegrees(360 * 0.5);

  private final DriveSubsystem driveSubsystem;
  private final DriveController controller;

  /** Creates a new TeleopDriveCommand. */
  public TeleopDriveCommand(DriveSubsystem driveSubsystem, DriveController controller) {
    this.driveSubsystem = driveSubsystem;
    this.controller = controller;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!DriverStation.isTeleopEnabled()) {
      return;
    }

    final var slowMode = controller.leftTrigger.get();
    final var robotRelative = controller.rightTrigger.get();

    var sidewaysPercentage = controller.getSidewaysPercentage();
    var forwardPercentage = controller.getForwardPercentage();
    var thetaPercentage = controller.getThetaPercentage();

    if (slowMode) {
      sidewaysPercentage *= 0.5;
      forwardPercentage *= 0.5;
      thetaPercentage *= 0.5;
    }

    driveSubsystem.driveTeleop(
        sidewaysPercentage, -forwardPercentage, thetaPercentage, !robotRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command never stops unless it's interrupted when another command is using the drivetrain
    return false;
  }
}
