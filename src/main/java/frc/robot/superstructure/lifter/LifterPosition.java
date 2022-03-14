// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.lifter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public enum LifterPosition {
  UP(Rotation2d.fromDegrees(120)),
  DOWN(Rotation2d.fromDegrees(70));

  public final TrapezoidProfile.State state;

  LifterPosition(Rotation2d angle) {
    this.state = new TrapezoidProfile.State(angle.getRadians(), 0);
  }
}
