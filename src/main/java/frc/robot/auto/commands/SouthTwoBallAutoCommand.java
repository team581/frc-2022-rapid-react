// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drive.DriveSubsystem;
import frc.robot.localization.Localization;
import frc.robot.localization.commands.SeedLocalizationCommand;
import frc.robot.superstructure.SuperstructureSubsystem;
import frc.robot.superstructure.commands.ArmDownAndSnarfCommand;
import frc.robot.superstructure.commands.ArmUpAndShootCommand;
import frc.robot.superstructure.commands.ArmUpAndStopCommand;
import lib.pathplanner.PPCommand;

/**
 * An autonomous routine starting in the south tarmac that scores the preload, snarfs the nearby
 * friendly ball, then returns to the fender to score it.
 */
public class SouthTwoBallAutoCommand extends SequentialCommandGroup {
  /** Creates a new SouthTwoBallAutoCommand. */
  public SouthTwoBallAutoCommand(
      DriveSubsystem driveSubsystem,
      SuperstructureSubsystem superstructure,
      Localization localization) {
    final var firstPath = PathPlanner.loadPath("SouthTwoBallAutoFirst", 4, 2.25);
    final var secondPath = PathPlanner.loadPath("SouthTwoBallAutoSecond", 4, 2.25);

    addCommands(
        new SeedLocalizationCommand(localization, firstPath),
        // Score preload
        new ArmUpAndShootCommand(superstructure),
        new ArmUpAndStopCommand(superstructure),
        // Fetch ball directly south of robot
        race(
            new PPCommand(firstPath, driveSubsystem, localization),
            new ArmDownAndSnarfCommand(superstructure)),
        // Travel back to fender
        new PPCommand(secondPath, driveSubsystem, localization),
        // Score second ball
        new ArmUpAndShootCommand(superstructure),
        new ArmUpAndStopCommand(superstructure));

    addRequirements(driveSubsystem, superstructure);
  }
}
