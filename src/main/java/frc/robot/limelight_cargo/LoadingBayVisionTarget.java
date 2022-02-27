// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelight_cargo;

import edu.wpi.first.math.util.Units;
import frc.robot.vision.LimelightVisionTarget;

/** The 2020 loading bay vision target. Exclusively used for debugging purposes. */
public class LoadingBayVisionTarget extends LimelightVisionTarget {
  public LoadingBayVisionTarget(CargoLimelightSubsystem limelight) {
    super(limelight, Units.inchesToMeters(11), CargoLimelightSubsystem.Pipelines.LOADING_BAY.index);
  }
}
