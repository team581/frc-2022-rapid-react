// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.UpperHubLimelightSubsystem;

/** The upper hub ring vision target. */
public class UpperHub extends LimelightVisionTarget {
  public UpperHub(UpperHubLimelightSubsystem limelight) {
    super(
        limelight,
        Units.feetToMeters(8) + Units.inchesToMeters(5.0 + (5.0 / 8.0)),
        Pipelines.UPPER_HUB);
  }
}
