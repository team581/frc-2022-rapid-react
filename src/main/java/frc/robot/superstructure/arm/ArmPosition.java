// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The goal positions for the arm to be in. Angles are CCW+. When the arm is level with the floor it
 * is at a rotation of 0. When the arm is perpendicular with the floor it is at a rotation of 90deg
 * (not -90deg or 270deg, that would be CW+).
 */
public enum ArmPosition {
  UP(Rotation2d.fromDegrees(30)),
  DOWN(Rotation2d.fromDegrees(-5));

  final TrapezoidProfile.State state;

  ArmPosition(Rotation2d angle) {
    this.state = new TrapezoidProfile.State(angle.getRadians(), 0);
  }
}
