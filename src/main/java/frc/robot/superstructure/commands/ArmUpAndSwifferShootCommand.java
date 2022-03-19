// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.superstructure.SuperstructureSubsystem;
import frc.robot.superstructure.arm.ArmPosition;
import frc.robot.superstructure.arm.commands.ArmCommand;
import frc.robot.superstructure.swiffer.commands.SwifferShootCommand;

/** Puts the arm up and shoots all cargo. */
public class ArmUpAndSwifferShootCommand extends SequentialCommandGroup {
  /** Creates a new ArmUpAndSwifferShootCommand. */
  public ArmUpAndSwifferShootCommand(SuperstructureSubsystem superstructure) {
    addCommands(
        // Arm up
        new ArmCommand(superstructure.arm, ArmPosition.UP),
        // Shoot all cargo after the arm is up
        new SwifferShootCommand(superstructure.swiffer));

    addRequirements(superstructure, superstructure.arm, superstructure.swiffer);
  }
}
