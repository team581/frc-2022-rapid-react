// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swiffer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.swiffer.SwifferMode;
import frc.robot.swiffer.SwifferSubsystem;

abstract class SwifferCommand extends CommandBase {
  private final SwifferSubsystem swiffer;
  private final SwifferMode mode;

  /** Creates a new SwifferCommand. */
  protected SwifferCommand(SwifferSubsystem swiffer, SwifferMode mode) {
    this.swiffer = swiffer;
    this.mode = mode;

    addRequirements(swiffer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swiffer.setDesiredMode(mode);
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
    return false;
  }
}
