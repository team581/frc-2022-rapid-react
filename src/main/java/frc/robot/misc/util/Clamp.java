// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.util;

import edu.wpi.first.math.MathUtil;

public class Clamp {
  public final double maximum;

  public Clamp(double maximum) {
    this.maximum = Math.abs(maximum);
  }

  public double clamp(double value) {
    return MathUtil.clamp(value, -maximum, maximum);
  }
}
