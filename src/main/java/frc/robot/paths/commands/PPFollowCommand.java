// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.paths.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drive.DriveSubsystem;
import lib.pathplanner.PPCommand;

/**
 * A command that follows the provided {@link PathPlannerTrajectory trajectory} using the {@link
 * DriveSubsystem drive subsystem's} odometry.
 */
public class PPFollowCommand extends SequentialCommandGroup {
  public PPFollowCommand(DriveSubsystem driveSubsystem, PathPlannerTrajectory trajectory) {
    addCommands(
        new InstantCommand(
            () -> driveSubsystem.resetSensorsForTrajectory(trajectory), driveSubsystem),
        new PPCommand(trajectory, driveSubsystem));
  }
}
