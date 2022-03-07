// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lifter.LifterPosition;
import frc.robot.lifter.LifterSubsystem;
import frc.robot.lifter.commands.LifterCommand;
import frc.robot.swiffer.SwifferSubsystem;
import frc.robot.swiffer.commands.SwifferShootCommand;
import frc.robot.swiffer.commands.SwifferStopCommand;

/** Puts the lifter up and shoots all cargo. */
public class LifterUpAndSwifferShootCommand extends SequentialCommandGroup {
  /** Creates a new LifterUpAndSwifferShootCommand. */
  public LifterUpAndSwifferShootCommand(SwifferSubsystem swiffer, LifterSubsystem lifter) {
    addCommands(
        // Lifter up
        new LifterCommand(lifter, LifterPosition.UP),
        // Shoot all cargo
        new SwifferShootCommand(swiffer),
        // Stop the flywheel once finished
        new SwifferStopCommand(swiffer));

    addRequirements(swiffer, lifter);
  }
}