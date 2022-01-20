// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import frc.robot.subsystems.LimelightSubsystem;

/** A vision target for the Limelight. */
public class LimelightVisionTarget extends VisionTarget {
  protected final LimelightSubsystem limelight;

  protected LimelightVisionTarget(LimelightSubsystem limelight) {
    this.limelight = limelight;
  }

  @Override
  public void onSelected() {
    limelight.setLEDMode(LimelightSubsystem.LEDMode.CURRENT_PIPELINE);
    limelight.setCamMode(LimelightSubsystem.CamMode.VISION_PROCESSOR);
  }
}
