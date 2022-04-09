// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drive.DriveSubsystem;
import frc.robot.localization.Localization;
import frc.robot.superstructure.SuperstructureSubsystem;
import frc.robot.superstructure.commands.ArmUpAndShootCommand;
import frc.robot.superstructure.commands.ArmUpAndStopCommand;

public class SingleBallAndSimpleTaxiCommand extends SequentialCommandGroup {
  /** Creates a new SingleBallAndSimpleTaxiCommand. */
  public SingleBallAndSimpleTaxiCommand(
      DriveSubsystem driveSubsystem,
      SuperstructureSubsystem superstructure,
      Localization localization) {
    addCommands(
        new ArmUpAndShootCommand(superstructure),
        new ArmUpAndStopCommand(superstructure),
        new TaxiCommand(driveSubsystem).withTimeout(4));

    addRequirements(driveSubsystem, superstructure);
  }
}
