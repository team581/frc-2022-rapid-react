// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Util functions for joysticks. */
public final class ControllerUtil {
  private static final double SCALAR = 3;

  private ControllerUtil() {}

  /** Scale a joystick value. */
  public static double joystickScale(double x) {
    return Math.signum(x) * (Math.pow(x, 2) / SCALAR);
  }
}
