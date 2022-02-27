// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.util;

public class Rotation3d {
  public final double pitch;
  public final double yaw;
  public final double roll;

  public Rotation3d(Number pitch, Number yaw, Number roll) {
    this.pitch = pitch.doubleValue();
    this.yaw = yaw.doubleValue();
    this.roll = roll.doubleValue();
  }
}
