// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelight_cargo;

import frc.robot.vision.LimelightVisionTarget;

/** A vision target for the 2022 cargo. */
public class CargoVisionTarget extends LimelightVisionTarget {
  /** The possible colors of cargo. */
  public enum Color {
    /** A red cargo. */
    RED,
    /** A blue cargo. */
    BLUE
  }

  public CargoVisionTarget(CargoLimelightSubsystem limeLight, Color color) {
    super(
        limeLight,
        // We are going to assume all cargo is on the floor (a height of 0)
        0.0,
        color == Color.RED
            ? CargoLimelightSubsystem.Pipelines.RED_CARGO.index
            : CargoLimelightSubsystem.Pipelines.BLUE_CARGO.index);
  }
}
