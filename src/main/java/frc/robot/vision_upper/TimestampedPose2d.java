// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_upper;

import edu.wpi.first.math.geometry.Pose2d;

/** A timestamped pose. */
public class TimestampedPose2d {
  public final Pose2d pose;
  public final double timestamp;

  public TimestampedPose2d(Pose2d pose, double timestamp) {
    this.pose = pose;
    this.timestamp = timestamp;
  }
}
