// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** A 2d translation in polar coordinates. */
public class PolarTranslation2d {
  private final double r;
  private final Rotation2d theta;

  public PolarTranslation2d() {
    r = 0;
    theta = new Rotation2d();
  }

  public PolarTranslation2d(double r, Rotation2d theta) {
    this.r = r;
    this.theta = theta;
  }

  public double getR() {
    return r;
  }

  public Rotation2d getTheta() {
    return theta;
  }

  public Translation2d getTranslation2d() {
    return new Translation2d(r, theta);
  }

  public PolarTranslation2d unaryMinus() {
    return new PolarTranslation2d(-r, theta.unaryMinus());
  }

  public PolarTranslation2d plus(PolarTranslation2d other) {
    return new PolarTranslation2d(r + other.r, theta.plus(other.theta));
  }

  public PolarTranslation2d minus(PolarTranslation2d other) {
    return new PolarTranslation2d(r - other.r, theta.minus(other.theta));
  }
}
