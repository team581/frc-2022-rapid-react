// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.util;

public class Vector3d {
  public final double x;
  public final double y;
  public final double z;

  public Vector3d(Number x, Number y, Number z) {
    this.x = x.doubleValue();
    this.y = y.doubleValue();
    this.z = z.doubleValue();
  }
}
