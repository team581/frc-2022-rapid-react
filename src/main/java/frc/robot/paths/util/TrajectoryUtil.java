// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.paths.util;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

public class TrajectoryUtil {
  /**
   * Get the pose that is the shortest distance to the current pose. This just compares the
   * translation component of a pose, the rotation part is ignored.
   */
  // TODO: Remove this once https://github.com/wpilibsuite/allwpilib/issues/4136 is implemented
  public static Pose2d shortestDistanceTo(Pose2d currentPose, List<Pose2d> otherPoses) {
    return otherPoses.stream()
        .min(
            (p1, p2) ->
                Double.compare(
                    currentPose.getTranslation().getDistance(p1.getTranslation()),
                    currentPose.getTranslation().getDistance(p2.getTranslation())))
        .orElse(null);
  }

  private TrajectoryUtil() {}
}
