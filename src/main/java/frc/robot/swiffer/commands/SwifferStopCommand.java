// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swiffer.commands;

import frc.robot.swiffer.SwifferMode;
import frc.robot.swiffer.SwifferSubsystem;

public class SwifferStopCommand extends SwifferCommand {
  private final SwifferSubsystem swiffer;

  /** Creates a new StopSwifferCommand. */
  public SwifferStopCommand(SwifferSubsystem swiffer) {
    super(swiffer, SwifferMode.STOPPED);

    this.swiffer = swiffer;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swiffer.atGoal(SwifferMode.STOPPED);
  }
}
