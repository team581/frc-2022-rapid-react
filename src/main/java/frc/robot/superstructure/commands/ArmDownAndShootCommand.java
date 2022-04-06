// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.superstructure.SuperstructureSubsystem;
import frc.robot.superstructure.arm.ArmPosition;
import frc.robot.superstructure.arm.commands.ArmCommand;
import frc.robot.superstructure.swiffer.SwifferMode;
import frc.robot.superstructure.swiffer.commands.SwifferShootCommand;

/**
 * Lowers the arm and shoots all cargo. This is used for resolving jams in the intake or discarding
 * unwanted cargo, not for scoring.
 */
public class ArmDownAndShootCommand extends SequentialCommandGroup {
  /** Creates a new ArmDownAndShootCommand. */
  public ArmDownAndShootCommand(SuperstructureSubsystem superstructure) {
    addCommands(
        // The shooting mode won't enable until the arm is in position so we manually tell
        // the lights that are are preparing to shoot.
        new InstantCommand(
            () -> superstructure.lights.setSubsystemState(SwifferMode.SHOOTING, false)),
        // Arm down
        new ArmCommand(superstructure.arm, ArmPosition.DOWN),
        // Shoot all cargo after the arm is down
        new SwifferShootCommand(superstructure.swiffer));

    addRequirements(superstructure, superstructure.arm, superstructure.swiffer);
  }
}
