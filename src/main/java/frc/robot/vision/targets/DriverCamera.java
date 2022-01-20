// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import frc.robot.subsystems.LimelightSubsystem;

/**
 * Not an actual vision target, selecting this this will enable raw camera output, turn off the
 * LEDs, etc. The camera will be used to help the driver see.
 */
public class DriverCamera extends VisionTarget {
  private final LimelightSubsystem limelight;

  public DriverCamera(LimelightSubsystem limelight) {
    this.limelight = limelight;
  }

  @Override
  public void onSelected() {
    limelight.setPipeline(9);
  }
}
