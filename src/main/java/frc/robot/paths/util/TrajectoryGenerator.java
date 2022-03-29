// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.paths.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.drive.DriveSubsystem;
import frc.robot.localization.Localization;
import java.util.List;

/** A helper class for generating {@link Trajectory trajectories}. */
public class TrajectoryGenerator {
  private final DriveSubsystem driveSubsystem;
  private final Localization localization;

  /** Creates a new TrajectoryGenerator. */
  public TrajectoryGenerator(DriveSubsystem driveSubsystem, Localization localization) {
    this.driveSubsystem = driveSubsystem;
    this.localization = localization;
  }

  /**
   * Generates a trajectory to go from your current position to a provided goal position. The
   * trajectory will also pass through several specified waypoints.
   *
   * @param goal The goal position
   * @param waypoints The waypoints to pass through before reaching the goal
   * @return A trajectory that will move the robot to the goal position
   */
  public Trajectory generateTrajectory(Pose2d goal, List<Translation2d> waypoints) {
    return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        localization.getPose(), waypoints, goal, driveSubsystem.trajectoryConfig);
  }

  /**
   * Generates a trajectory to go directly from your current position to a provided goal position.
   *
   * @param goal The goal position
   * @return A trajectory that will move the robot to the goal position
   */
  public Trajectory generateTrajectory(Pose2d goal) {
    return generateTrajectory(goal, List.of());
  }
}
