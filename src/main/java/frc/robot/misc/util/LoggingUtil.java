// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class LoggingUtil {
  private LoggingUtil() {}

  public static double[] poseToArray(Pose2d pose) {
    return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
  }

  public static double[] translationToArray(Translation2d translation) {
    return new double[] {translation.getX(), translation.getY()};
  }
}
