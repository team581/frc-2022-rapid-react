// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.lifter.LifterPosition;
import frc.robot.lifter.LifterSubsystem;
import frc.robot.lifter.commands.LifterCommand;
import frc.robot.swiffer.SwifferSubsystem;
import frc.robot.swiffer.commands.SwifferStopCommand;

/** Lift the swiffer up and stop the flywheel. */
public class LifterUpAndSwifferStopCommand extends ParallelCommandGroup {
  /** Creates a new LifterUpAndSwifferStopCommand. */
  public LifterUpAndSwifferStopCommand(SwifferSubsystem swiffer, LifterSubsystem lifter) {
    addCommands(new LifterCommand(lifter, LifterPosition.UP), new SwifferStopCommand(swiffer));

    addRequirements(swiffer, lifter);
  }
}
