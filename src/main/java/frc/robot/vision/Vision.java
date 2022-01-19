// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.vision.targets.UpperHub;

/** Used for configuring the vision system. */
public class Vision {
  private final LimelightSubsystem limelightSubsystem;

  public enum Mode {
    /** Computer vision and lights are enabled. */
    COMPUTER_VISION,
    /** Computer vision is not enabled. The LEDs are disabled and a raw camera feed is shown. */
    RAW_VIDEO
  }

  public Vision(LimelightSubsystem limelightSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
  }

  public void setMode(Mode mode) {
    switch (mode) {
      case COMPUTER_VISION:
        // TODO: This only works because 2022 has a single vision target, in games with multiple
        // ones you will need to rewrite this code
        limelightSubsystem.setPipeline(UpperHub.getInstance().getPipeline());
        break;
      case RAW_VIDEO:
        limelightSubsystem.setPipeline(9);
        break;
      default:
        throw new IllegalArgumentException("Invalid mode: " + mode);
    }
  }
}
