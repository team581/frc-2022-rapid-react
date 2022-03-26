// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** A 2d pose in polar coordinates. */
public class PolarPose2d {
  private final double r;
  private final Rotation2d theta;

  public PolarPose2d() {
    r = 0;
    theta = new Rotation2d();
  }

  public PolarPose2d(double r, Rotation2d theta) {
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

  public PolarPose2d unaryMinus() {
    return new PolarPose2d(-r, theta.unaryMinus());
  }
}
