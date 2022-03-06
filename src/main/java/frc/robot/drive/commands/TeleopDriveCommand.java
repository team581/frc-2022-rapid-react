// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controller.DriveController;
import frc.robot.drive.DriveSubsystem;

public class TeleopDriveCommand extends CommandBase {
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

    final var x = controller.getXPercentage();
    final var y = controller.getYPercentage();
    final var theta = controller.getThetaPercentage();

    driveSubsystem.driveTeleop(x, -y, theta);
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
