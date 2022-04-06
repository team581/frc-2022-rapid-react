// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer;

import edu.wpi.first.math.util.Units;

/** A mode that the swiffer flywheel can be in. */
public enum SwifferMode {
  STOPPED(0),
  SNARFING(900),
  SHOOTING(-1000);

  /** Angular velocity in radians/second. */
  final double angularVelocity;

  /** @param rpm The desired rotations per minute (RPM) for this mode. */
  SwifferMode(double rpm) {
    this.angularVelocity = Units.rotationsPerMinuteToRadiansPerSecond(rpm);
  }
}
