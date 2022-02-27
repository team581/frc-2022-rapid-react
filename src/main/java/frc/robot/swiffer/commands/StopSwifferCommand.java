// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swiffer.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.swiffer.SwifferSubsystem;

/** A command that stops the swiffer from shooting or snarfing. */
public class StopSwifferCommand extends InstantCommand {
  private final SwifferSubsystem swiffer;

  public StopSwifferCommand(SwifferSubsystem swiffer) {
    // setting this command's swiffer to the one provided
    this.swiffer = swiffer;

    // makes it where one command can use the subsystem at a time
    addRequirements(swiffer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // just tells the swiffer to stop
    swiffer.stop();
  }
}
