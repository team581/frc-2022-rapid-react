// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import frc.robot.subsystems.CargoLimelightSubsystem;

/** A vision target for the 2022 cargo. */
public class Cargo extends LimelightVisionTarget {
  /** The possible colors of cargo. */
  public enum Color {
    /** A red cargo. */
    RED,
    /** A blue cargo. */
    BLUE
  }

  public Cargo(CargoLimelightSubsystem limeLight, Color color) {
    super(
        limeLight,
        // We are going to assume all cargo is on the floor (a height of 0)
        0.0,
        color == Color.RED ? Pipelines.RED_CARGO : Pipelines.BLUE_CARGO);
  }
}
