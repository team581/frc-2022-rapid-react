// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import frc.robot.subsystems.LimelightSubsystem;

/** The 2020 power port vision target. Exclusively used for debugging purposes. */
public class PowerPort extends LimelightVisionTarget {
  private final LimelightSubsystem limelight;

  public PowerPort(LimelightSubsystem limelight) {
    this.limelight = limelight;
  }

  @Override
  public void onSelected() {
    limelight.setCamMode(LimelightSubsystem.CamMode.VISION_PROCESSOR);
    limelight.setPipeline(1);
  }
}
