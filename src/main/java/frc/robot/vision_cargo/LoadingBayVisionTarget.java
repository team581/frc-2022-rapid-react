// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_cargo;

import edu.wpi.first.math.util.Units;
import frc.robot.vision.VisionTarget;

/** The 2020 loading bay vision target. Exclusively used for debugging purposes. */
// TODO: Delete this
public class LoadingBayVisionTarget extends VisionTarget {
  public LoadingBayVisionTarget(CargoVisionSubsystem vision) {
    super(vision, Units.inchesToMeters(11), CargoVisionSubsystem.Pipelines.LOADING_BAY.index);
  }
}
