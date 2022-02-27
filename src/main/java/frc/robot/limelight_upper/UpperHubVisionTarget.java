// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelight_upper;

import edu.wpi.first.math.util.Units;
import frc.robot.vision.LimelightVisionTarget;

/** The upper hub ring vision target. */
public class UpperHubVisionTarget extends LimelightVisionTarget {
  public UpperHubVisionTarget(UpperHubLimelightSubsystem limelight) {
    super(
        limelight,
        Units.feetToMeters(8) + Units.inchesToMeters(5.0 + (5.0 / 8.0)),
        UpperHubLimelightSubsystem.Pipelines.UPPER_HUB.index);
  }
}
