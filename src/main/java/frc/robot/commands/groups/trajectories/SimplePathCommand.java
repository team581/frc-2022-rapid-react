// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups.trajectories;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.util.TrajectoryCommandFactory;
import frc.robot.paths.PPPaths;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A command that follows the SimplePath created with PathPlanner using the robot's odometry for
 * localization.
 */
public class SimplePathCommand extends SequentialCommandGroup {
  public SimplePathCommand(DriveSubsystem driveSubsystem) {
    final var commandFactory = new TrajectoryCommandFactory(driveSubsystem);

    addCommands(commandFactory.createPPCommand(PPPaths.simplePath, driveSubsystem::getPose));
  }
}
