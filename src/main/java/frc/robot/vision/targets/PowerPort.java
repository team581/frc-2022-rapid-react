// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

public class PowerPort extends VisionTarget {
  private static final PowerPort instance = new PowerPort();

  public static PowerPort getInstance() {
    return instance;
  }

  private PowerPort() {}

  @Override
  public int getPipeline() {
    return 1;
  }
}
