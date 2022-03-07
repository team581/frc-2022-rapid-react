// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer.commands;

import frc.robot.superstructure.swiffer.Swiffer;
import frc.robot.superstructure.swiffer.SwifferMode;

public class SwifferStopCommand extends SwifferCommand {
  private final Swiffer swiffer;

  /** Creates a new StopSwifferCommand. */
  public SwifferStopCommand(Swiffer swiffer) {
    super(swiffer, SwifferMode.STOPPED);

    this.swiffer = swiffer;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swiffer.atGoal(SwifferMode.STOPPED);
  }
}
