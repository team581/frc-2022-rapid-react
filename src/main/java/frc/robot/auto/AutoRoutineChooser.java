// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.commands.SingleBallAndSimpleTaxiCommand;
import frc.robot.drive.DriveSubsystem;
import frc.robot.localization.Localization;
import frc.robot.superstructure.SuperstructureSubsystem;

/** Chooses the autonomous routine to run at the start of a match. */
public class AutoRoutineChooser {
  private final SuperstructureSubsystem superstructure;
  private final DriveSubsystem driveSubsystem;
  private final Localization localization;

  public AutoRoutineChooser(
      DriveSubsystem driveSubsystem,
      SuperstructureSubsystem superstructureSubsystem,
      Localization localization) {
    this.driveSubsystem = driveSubsystem;
    superstructure = superstructureSubsystem;
    this.localization = localization;
  }

  /** Get the {@link Command command} to run in autonomous. */
  public Command getAutonomousCommand() {
    return new SingleBallAndSimpleTaxiCommand(driveSubsystem, superstructure, localization)
        .withTimeout(5);
  }
}
