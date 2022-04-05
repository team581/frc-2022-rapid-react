// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.localization.Localization;

/** Seed the localization system's pose. */
public class SeedLocalizationCommand extends InstantCommand {
  private final Pose2d robotPose;
  private final Localization localization;

  /** Seed the localization system's pose using the provided pose. */
  public SeedLocalizationCommand(Localization localization, Pose2d robotPose) {
    this.localization = localization;
    this.robotPose = robotPose;
  }

  /** Seed the localization system's pose using the first pose of a trajectory. */
  public SeedLocalizationCommand(Localization localization, Trajectory trajectory) {
    this(localization, trajectory.getInitialPose());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    localization.resetPose(robotPose);
  }
}
