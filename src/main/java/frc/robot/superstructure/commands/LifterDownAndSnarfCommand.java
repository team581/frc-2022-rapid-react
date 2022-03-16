// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.superstructure.SuperstructureSubsystem;
import frc.robot.superstructure.lifter.LifterPosition;
import frc.robot.superstructure.lifter.commands.LifterCommand;
import frc.robot.superstructure.swiffer.commands.SwifferSnarfCommand;

/** Lowers the swiffer while snarfing. */
public class LifterDownAndSnarfCommand extends ParallelCommandGroup {
  /** Creates a new LifterDownAndSnarfCommand. */
  public LifterDownAndSnarfCommand(SuperstructureSubsystem superstructure) {
    addCommands(
        // Start lowering the lifter
        new LifterCommand(superstructure.lifter, LifterPosition.DOWN),
        // While that's happening we begin spinning up the swiffer
        new SwifferSnarfCommand(superstructure.swiffer));

    addRequirements(superstructure, superstructure.lifter, superstructure.swiffer);
  }
}
