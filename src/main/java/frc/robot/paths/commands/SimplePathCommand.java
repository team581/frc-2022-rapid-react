// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.paths.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drive.DriveSubsystem;
import frc.robot.paths.PPPaths;
import lib.pathplanner.PPCommand;

/**
 * A command that follows the SimplePath created with PathPlanner using the robot's odometry for
 * localization.
 */
public class SimplePathCommand extends SequentialCommandGroup {
  public SimplePathCommand(DriveSubsystem driveSubsystem) {

    addCommands(new PPCommand(PPPaths.simplePath, driveSubsystem));
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }
}
