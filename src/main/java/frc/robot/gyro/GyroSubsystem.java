// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.gyro.GyroIO.Inputs;
import org.littletonrobotics.junction.Logger;

public class GyroSubsystem extends SubsystemBase {
  private final GyroIO io;
  private final Inputs inputs = new Inputs();

  /** Creates a new GyroSubsystem. */
  public GyroSubsystem(GyroIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Gyro", inputs);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    io.zeroHeading();
  }

  /** Returns the turn rate of the robot in radians/second. */
  public Rotation2d getTurnRate() {
    return new Rotation2d(inputs.turnRateRadiansPerSecond);
  }

  /** Get the heading of the robot. */
  public Rotation2d getRotation() {
    return new Rotation2d(inputs.rotationRadians);
  }
}
