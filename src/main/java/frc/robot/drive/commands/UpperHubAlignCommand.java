// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import frc.robot.drive.DriveSubsystem;
import frc.robot.localization.Localization;
import frc.robot.paths.commands.DynamicTrajectoryFollowCommand;
import frc.robot.paths.util.TrajectoryGenerator;
import frc.robot.vision_cargo.UpperHubVisionTarget;
import java.util.List;

public class UpperHubAlignCommand extends DynamicTrajectoryFollowCommand {
  /**
   * The number of meters that we want to be away from the center of the hub in order to be properly
   * lined up with the fenders to score.
   */
  private static final double GOAL_DISTANCE_FROM_HUB_CENTER =
      UpperHubVisionTarget.RADIUS + Units.feetToMeters(2);

  /** Four poses that align you with the fenders while facing toward the center of the hub. */
  // TODO: The hub is not perfectly aligned, it is slightly rotated. Refer to the field diagram and
  // update these goal poses to properly align
  // https://firstfrc.blob.core.windows.net/frc2022/FieldAssets/2022LayoutMarkingDiagram.pdf
  private static final List<Pose2d> GOAL_POSES =
      List.of(
          // Right
          new Pose2d(
              UpperHubVisionTarget.POSE.plus(new Translation2d(GOAL_DISTANCE_FROM_HUB_CENTER, 0)),
              Rotation2d.fromDegrees(180)),
          // Left
          new Pose2d(
              UpperHubVisionTarget.POSE.minus(new Translation2d(GOAL_DISTANCE_FROM_HUB_CENTER, 0)),
              Rotation2d.fromDegrees(0)),
          // Up
          new Pose2d(
              UpperHubVisionTarget.POSE.plus(new Translation2d(0, GOAL_DISTANCE_FROM_HUB_CENTER)),
              Rotation2d.fromDegrees(90)),
          // Down
          new Pose2d(
              UpperHubVisionTarget.POSE.minus(new Translation2d(0, GOAL_DISTANCE_FROM_HUB_CENTER)),
              Rotation2d.fromDegrees(270)));

  private static Pose2d chooseGoalPose(Localization localization) {
    return frc.robot.paths.util.TrajectoryUtil.shortestDistanceTo(
        localization.getPose(), GOAL_POSES);
  }

  private static Trajectory chooseTrajectory(
      DriveSubsystem driveSubsystem, Localization localization) {
    return new TrajectoryGenerator(driveSubsystem, localization)
        .generateTrajectory(chooseGoalPose(localization));
  }

  public UpperHubAlignCommand(DriveSubsystem driveSubsystem, Localization localization) {
    super(
        () -> chooseTrajectory(driveSubsystem, localization),
        localization,
        () -> chooseGoalPose(localization).getRotation(),
        driveSubsystem);
  }
}
