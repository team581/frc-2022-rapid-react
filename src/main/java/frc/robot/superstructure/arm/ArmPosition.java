// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public enum ArmPosition {
  UP(Rotation2d.fromDegrees(121.904)),
  DOWN(Rotation2d.fromDegrees(0));

  public final TrapezoidProfile.State state;

  ArmPosition(Rotation2d angle) {
    this.state = new TrapezoidProfile.State(angle.getRadians(), 0);
  }
}
