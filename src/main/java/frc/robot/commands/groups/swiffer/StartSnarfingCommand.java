// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups.swiffer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwifferSubsystem;

/** A command that starts the snarfer and waits for it to speed up. */
public class StartSnarfingCommand extends SequentialCommandGroup {
  /** Creates a new StartSnarfingCommand. */
  public StartSnarfingCommand(
      // gives the subsystem to use
      SwifferSubsystem swiffer) {
    addCommands(
        new InstantCommand(
            // creating an instant command that starts snarfing (swiffer is the command that is
            // snarfing)
            swiffer::startSnarfing),
        // TODO Measure the number to actually wait
        // Created a wait command to wait for the snarfer to spin up
        new WaitCommand(0.5));
  }
}
