// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_upper;

import frc.robot.vision.VisionSubsystemBase;

public class UpperHubVisionSubsystem extends VisionSubsystemBase {
  public enum Pipelines {
    UPPER_HUB(0),
    DRIVER_MODE(9);

    public final int index;

    Pipelines(final int index) {
      this.index = index;
    }
  }

  public final UpperHubVisionTarget upperHub = new UpperHubVisionTarget(this);

  /** Creates a new UpperHubVisionSubsystem. */
  public UpperHubVisionSubsystem(UpperHubVisionIO io) {
    // TODO: Add in the camera's angle of elevation and height
    super("UpperVision", io, -1.0, -1.0, Pipelines.DRIVER_MODE.index);
  }
}
