// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_cargo;

import frc.robot.vision.VisionTarget;

/** A vision target for the 2022 cargo. */
public class CargoVisionTarget extends VisionTarget {
  /** The possible colors of cargo. */
  public enum Color {
    /** A red cargo. */
    RED,
    /** A blue cargo. */
    BLUE
  }

  public CargoVisionTarget(CargoVisionSubsystem vision, Color color) {
    super(
        vision,
        // We are going to assume all cargo is on the floor (a height of 0)
        0,
        color == Color.RED
            ? CargoVisionSubsystem.Pipelines.RED_CARGO.index
            : CargoVisionSubsystem.Pipelines.BLUE_CARGO.index);
  }
}
