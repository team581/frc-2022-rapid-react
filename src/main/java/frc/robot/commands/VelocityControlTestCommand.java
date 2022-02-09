// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.InputFilter;

public class VelocityControlTestCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final InputFilter inputFilter;

  /** Creates a new VelocityControlTestCommand. */
  public VelocityControlTestCommand(DriveSubsystem driveSubsystem, InputFilter inputFilter) {
    addRequirements(driveSubsystem);

    this.driveSubsystem = driveSubsystem;
    this.inputFilter = inputFilter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inputFilter.useCargoControl();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.driveWithSpeeds(new ChassisSpeeds(1.5, 0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveWithSpeeds(new ChassisSpeeds(0, 0, 0));
    driveSubsystem.stopMotors();
    inputFilter.useDriverControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
