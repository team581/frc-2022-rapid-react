// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.superstructure.lifter.LifterPosition;
import frc.robot.superstructure.lifter.LifterSubsystem;
import frc.robot.superstructure.lifter.commands.LifterCommand;
import frc.robot.superstructure.swiffer.SwifferSubsystem;
import frc.robot.superstructure.swiffer.commands.SwifferSnarfCommand;

/** Lowers the swiffer while snarfing. Once finished the swiffer will stop and be lifted up. */
public class LifterDownAndSnarfCommand extends SequentialCommandGroup {
  /** Creates a new LifterDownAndSnarfCommand. */
  public LifterDownAndSnarfCommand(SwifferSubsystem swiffer, LifterSubsystem lifter) {
    addCommands(
        // Start snarfing while the lifter is lowered
        parallel(new LifterCommand(lifter, LifterPosition.DOWN), new SwifferSnarfCommand(swiffer)),
        // Once finished, put the lifter up and stop the swiffer flywheel
        new LifterUpAndSwifferStopCommand(swiffer, lifter));

    addRequirements(swiffer, lifter);
  }
}
