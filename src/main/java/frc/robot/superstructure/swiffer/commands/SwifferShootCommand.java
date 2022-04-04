// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.superstructure.swiffer.Swiffer;
import frc.robot.superstructure.swiffer.SwifferMode;

public class SwifferShootCommand extends SwifferCommand {
  private final Timer timer = new Timer();

  /** Creates a new SwifferShootCommand. */
  public SwifferShootCommand(Swiffer swiffer) {
    super(swiffer, SwifferMode.SHOOTING);
  }

  @Override
  public void initialize() {
    super.initialize();

    timer.reset();
    timer.start();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: Check if 0 cargo are being held and use that to determine if finished
    return timer.hasElapsed(1.5);
  }
}
