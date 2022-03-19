// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.superstructure.SuperstructureSubsystem;
import frc.robot.superstructure.arm.ArmPosition;
import frc.robot.superstructure.arm.commands.ArmCommand;
import frc.robot.superstructure.swiffer.commands.SwifferStopCommand;

/** Lift the swiffer up and stop the flywheel. */
public class ArmUpAndSwifferStopCommand extends ParallelCommandGroup {
  /** Creates a new ArmUpAndSwifferStopCommand. */
  public ArmUpAndSwifferStopCommand(SuperstructureSubsystem superstructure) {
    addCommands(
        // Arm up
        new ArmCommand(superstructure.arm, ArmPosition.UP),
        // Stop the flywheel
        new SwifferStopCommand(superstructure.swiffer));

    addRequirements(superstructure, superstructure.arm, superstructure.swiffer);
  }
}
