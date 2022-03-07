// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lifter;

import edu.wpi.first.math.geometry.Rotation2d;

public enum Position {
  UP(Rotation2d.fromDegrees(60)),
  DOWN(new Rotation2d(0));

  public final Rotation2d angle;

  Position(Rotation2d angle) {
    this.angle = angle;
  }
}
