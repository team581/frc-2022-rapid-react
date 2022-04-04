// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.util;

import edu.wpi.first.math.geometry.Rotation2d;

/** Used for doing math with circles. */
public class WheelConverter {
  public static WheelConverter fromRadius(double radius) {
    return new WheelConverter(radius);
  }

  public static WheelConverter fromDiameter(double diameter) {
    return new WheelConverter(diameter / 2);
  }

  private final double radius;

  private WheelConverter(double radius) {
    this.radius = radius;
  }

  /** Converts a rotation in radians to a distance. */
  public double radiansToDistance(double radians) {
    return radians * radius;
  }

  /** Converts a rotation to a distance. */
  public double radiansToDistance(Rotation2d rotation) {
    return radiansToDistance(rotation.getRadians());
  }

  /** Converts a distance to a rotation in radians. */
  public double distanceToRadians(double distance) {
    return distance / radius;
  }
}
