// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.paths;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.subsystems.drive.Drivebase;

/**
 * Stores {@link PathPlannerTrajectory PathPlanner paths}.
 *
 * @see https://github.com/mjansen4857/pathplanner
 */
public class PPPaths {
  private static final class Constants {
    public static final double MAX_VELOCITY = Drivebase.Constants.MAX_VELOCITY;
    public static final double MAX_ACCELERATION = Drivebase.Constants.MAX_ACCELERATION;
  }

  private PPPaths() {}

  public static final PathPlannerTrajectory simplePath =
      PathPlanner.loadPath("SimplePath", Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION);
}
