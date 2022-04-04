// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drive.DriveSubsystem;
import frc.robot.localization.Localization;
import frc.robot.superstructure.SuperstructureSubsystem;
import frc.robot.superstructure.commands.ArmUpAndSwifferShootCommand;
import frc.robot.superstructure.commands.ArmUpAndSwifferStopCommand;
import lib.pathplanner.PPCommand;

public class SouthSingleBallAuto extends SequentialCommandGroup {
  /** Creates a new SouthSingleBallAuto. */
  public SouthSingleBallAuto(
      DriveSubsystem driveSubsystem,
      SuperstructureSubsystem superstructure,
      Localization localization) {
    final var path = PathPlanner.loadPath("SouthSingleBallAuto", 3, 2.5);

    addCommands(
        new InstantCommand(() -> localization.resetPose(path.getInitialPose())),
        new ArmUpAndSwifferShootCommand(superstructure),
        new ArmUpAndSwifferStopCommand(superstructure),
        new PPCommand(path, driveSubsystem, localization));

    addRequirements(driveSubsystem, superstructure);
  }
}
