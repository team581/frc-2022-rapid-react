// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

/** Used for configuring the vision system. */
public class Vision {
  public enum Mode {
    /** Vision is enabled and trying to detect the high target (upper hub). */
    HIGH_TARGET,
    /** Vision is not enabled. The LEDs are disabled and a raw camera feed is shown. */
    RAW_VIDEO
  }

  private Vision() {}

  public static void setMode(Mode mode) {
    switch (mode) {
      case HIGH_TARGET:
        Limelight.setPipeline(0);
        Limelight.setCamMode(Limelight.CamMode.VISION_PROCESSOR);
        break;
      case RAW_VIDEO:
        Limelight.setPipeline(9);
        Limelight.setCamMode(Limelight.CamMode.DRIVER_CAMERA);
        break;
      default:
        throw new IllegalArgumentException("Invalid mode: " + mode);
    }
  }
}
