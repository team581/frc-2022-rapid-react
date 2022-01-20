// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

public class Position3d {
  public final Vector3d translation;
  public final Rotation3d rotation;

  public Position3d(Vector3d translation, Rotation3d rotation) {
    this.translation = translation;
    this.rotation = rotation;
  }
}
