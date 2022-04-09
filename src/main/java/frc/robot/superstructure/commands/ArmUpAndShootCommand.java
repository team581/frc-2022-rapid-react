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
import frc.robot.superstructure.swiffer.commands.SwifferCommand;

/** Puts the arm up and shoots all cargo. */
public class ArmUpAndShootCommand extends SequentialCommandGroup {
  /**
   * The maximum amount of time (in seconds) it will take to shoot any amount of cargo from the SPU.
   * If we had cargo sensors we'd use those instead of just relying on a timer.
   */
  private static final double SHOOT_DURATION = 1;

  /** Creates a new ArmUpAndShootCommand. */
  public ArmUpAndShootCommand(SuperstructureSubsystem superstructure) {
    addCommands(
        // The shooting mode won't enable until the arm is in position so we manually tell
        // the lights that are are preparing to shoot.
        new InstantCommand(
            () -> superstructure.lights.setSubsystemState(SwifferMode.SHOOTING, false)),
        // Arm up
        new ArmCommand(superstructure.arm, ArmPosition.UP),
        // Shoot all cargo after the arm is up
        new SwifferCommand(superstructure.swiffer, SwifferMode.SHOOTING)
            .withTimeout(SHOOT_DURATION));

    addRequirements(superstructure, superstructure.arm, superstructure.swiffer);
  }
}
