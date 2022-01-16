// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

public class UpperHub extends VisionTarget {
  private static final UpperHub instance = new UpperHub();

  public static UpperHub getInstance() {
    return instance;
  }

  private UpperHub() {}

  @Override
  public int getPipeline() {
    return 0;
  }
}
