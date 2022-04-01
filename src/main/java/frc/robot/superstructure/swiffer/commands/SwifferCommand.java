// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.superstructure.swiffer.Swiffer;
import frc.robot.superstructure.swiffer.SwifferMode;

public class SwifferCommand extends CommandBase {
  private final Swiffer swiffer;
  private final SwifferMode mode;

  /** Creates a new SwifferCommand. */
  public SwifferCommand(Swiffer swiffer, SwifferMode mode) {
    this.swiffer = swiffer;
    this.mode = mode;

    addRequirements(swiffer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swiffer.setDesiredMode(mode);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
