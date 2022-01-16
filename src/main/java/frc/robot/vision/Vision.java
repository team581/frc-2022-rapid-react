// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

/** Used for configuring the vision system. */
public class Vision {
  public enum Mode {
    /** Computer vision and lights are enabled. */
    COMPUTER_VISION,
    /** Computer vision is not enabled. The LEDs are disabled and a raw camera feed is shown. */
    RAW_VIDEO
  }

  private Vision() {}

  public static void setMode(Mode mode) {
    switch (mode) {
      case COMPUTER_VISION:
        Limelight.setCamMode(Limelight.CamMode.VISION_PROCESSOR);
        Limelight.setLEDMode(Limelight.LEDMode.CURRENT_PIPELINE);
        break;
      case RAW_VIDEO:
        Limelight.setCamMode(Limelight.CamMode.DRIVER_CAMERA);
        Limelight.setLEDMode(Limelight.LEDMode.OFF);
        break;
      default:
        throw new IllegalArgumentException("Invalid mode: " + mode);
    }
  }
}
