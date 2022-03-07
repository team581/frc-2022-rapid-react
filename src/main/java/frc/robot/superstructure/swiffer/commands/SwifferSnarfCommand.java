// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer.commands;

import frc.robot.superstructure.swiffer.SwifferMode;
import frc.robot.superstructure.swiffer.SwifferSubsystem;

public class SwifferSnarfCommand extends SwifferCommand {
  /** Creates a new SwifferSnarfCommand. */
  public SwifferSnarfCommand(SwifferSubsystem swiffer) {
    super(swiffer, SwifferMode.SNARFING);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: Check if 2 cargo are being held and use that to determine if finished
    return false;
  }
}
