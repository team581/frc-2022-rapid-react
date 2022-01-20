// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import frc.lib.limelight.Limelight;
import frc.robot.subsystems.LimelightSubsystem;

/** A vision target for the Limelight. */
public class LimelightVisionTarget extends VisionTarget {
  protected final Limelight limelight;

  protected LimelightVisionTarget(LimelightSubsystem limelightSubsystem) {
    this.limelight = limelightSubsystem.limelight;
  }

  @Override
  public void onSelected() {
    limelight.setLEDMode(Limelight.LEDMode.CURRENT_PIPELINE);
    limelight.setCamMode(Limelight.CamMode.VISION_PROCESSOR);
  }
}
