// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.superstructure.arm.Arm;
import frc.robot.superstructure.arm.ArmPosition;

/**
 * A command to move the arm to a desired position. Finishes after the arm has first met the desired
 * position.
 */
public class ArmCommand extends CommandBase {
  private final Arm arm;
  private final ArmPosition goalPosition;

  /** Creates a new ArmCommand. */
  public ArmCommand(Arm arm, ArmPosition goalPosition) {
    this.arm = arm;
    this.goalPosition = goalPosition;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setDesiredPosition(goalPosition);
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
    // This exits the command when the arm is at the desired position. This does not turn off the
    // arm, it will continue to maintain the desired position set by this command. Arguably this
    // command should never be "finished", but it's more convenient to do it like this.
    return arm.atPosition(goalPosition);
  }
}
