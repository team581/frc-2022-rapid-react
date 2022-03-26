// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.drive.DriveSubsystem;
import frc.robot.vision.VisionTarget;

/**
 * A helper class for generating {@link Trajectory trajectories} to align with a {@link VisionTarget
 * vision target}.
 */
public class VisionSystemTrajectoryGenerator {
  private final DriveSubsystem driveSubsystem;

  /**
   * Creates a new LimelightTrajectoryGenerator.
   *
   * @param driveSubsystem Your drive subsystem
   */
  public VisionSystemTrajectoryGenerator(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  /**
   * Generates a trajectory to go from your current position to a provided goal position by using
   * the vision system to get your current position.
   *
   * @param target The vision target you want to align with
   * @param goal The goal position relative to the vision target
   * @return A trajectory that will move the robot to the goal position
   */
  public Trajectory generateTrajectory(VisionTarget target, Pose2d goal) {
    // TODO: Refactor once pose estimation is implemented
    return null;
  }
}
