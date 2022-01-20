// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import frc.robot.subsystems.PhotonVisionSubsystem;

/** A vision target for the 2022 cargo. */
public class Cargo extends PhotonVisionVisionTarget {
  /** The possible colors of cargo. */
  public enum Color {
    /** A red cargo. */
    RED,
    /** A blue cargo. */
    BLUE
  }

  private final Color color;

  public Cargo(PhotonVisionSubsystem photonVision, Color color) {
    super(photonVision);

    this.color = color;
  }

  @Override
  public void onSelected() {
    super.onSelected();

    switch (color) {
      case RED:
        photonVision.camera.setPipelineIndex(0);
        break;
      case BLUE:
        photonVision.camera.setPipelineIndex(1);
        break;
      default:
        throw new IllegalArgumentException("Unknown color: " + color);
    }
  }
}
