// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.superstructure.swiffer.Swiffer;
import frc.robot.superstructure.swiffer.SwifferMode;

abstract class SwifferCommand extends CommandBase {
  private final Swiffer swiffer;
  private final SwifferMode mode;

  /** Creates a new SwifferCommand. */
  protected SwifferCommand(Swiffer swiffer, SwifferMode mode) {
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
  // This is abstract to ensure you don't forget to override the default `Command interface behavior
  // of `return false`
  @Override
  public abstract boolean isFinished();
}
