// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import frc.robot.subsystems.LimelightSubsystem;

/** A vision target for the 2022 cargo. */
public class Cargo extends LimelightVisionTarget {
  /** The possible colors of cargo. */
  public enum Color {
    /** A red cargo. */
    RED,
    /** A blue cargo. */
    BLUE
  }

  private final Color color;

  public Cargo(LimelightSubsystem limeLight, Color color) {
    super(limeLight, 0.0);

    this.color = color;
  }

  @Override
  public void prepareForUse() {
    super.prepareForUse();

    switch (color) {
      case RED:
        limelightSubsystem.limelight.setPipeline(3);
        break;
      case BLUE:
        limelightSubsystem.limelight.setPipeline(4);
        break;
      default:
        throw new IllegalArgumentException("Unknown color: " + color);
    }
  }
}
