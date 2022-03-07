// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer;

/** A mode that the swiffer flywheel can be in. */
public enum SwifferMode {
  STOPPED(0),
  SNARFING(500),
  SHOOTING(250);

  /** Rotations per minute. */
  public final double rpm;

  SwifferMode(double rpm) {
    this.rpm = rpm;
  }
}
