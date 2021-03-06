// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.imu.ImuIO.Inputs;
import org.littletonrobotics.junction.Logger;

public class ImuSubsystem extends SubsystemBase {
  private final ImuIO io;
  private final Inputs inputs = new Inputs();
  private Rotation2d offset = new Rotation2d();

  /** Creates a new ImuSubsystem. */
  public ImuSubsystem(ImuIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Imu", inputs);
    Logger.getInstance().recordOutput("Imu/OffsettedPositionRadians", getRotation().getRadians());
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    resetHeadingTo(new Rotation2d());
  }

  /** Sets the heading of the IMU. */
  public void resetHeadingTo(Rotation2d heading) {
    io.zeroHeading();
    offset = heading;
  }

  /** Returns the turn rate of the robot in radians/second. */
  public Rotation2d getTurnRate() {
    return new Rotation2d(inputs.turnRateRadiansPerSecond);
  }

  /** Get the heading of the robot. */
  public Rotation2d getRotation() {
    return new Rotation2d(inputs.rotationRadians - offset.getRadians());
  }
}
