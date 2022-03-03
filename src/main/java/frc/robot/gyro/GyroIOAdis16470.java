// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.gyro;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class GyroIOAdis16470 implements GyroIO {
  private final ADIS16470_IMU sensor;

  public GyroIOAdis16470() {
    switch (frc.robot.Constants.getRobot()) {
      case TEST_2020_BOT:
      case SIM_BOT:
        sensor = new ADIS16470_IMU();
        break;
      default:
        throw new IllegalStateException(
            "The currently configured robot doesn't support this subsystem");
    }
  }

  @Override
  public void zeroHeading() {
    sensor.reset();
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.tempCelcius = 0;
    inputs.rotationRadians = Units.degreesToRadians(-sensor.getAngle());
    inputs.turnRateRadiansPerSecond = Units.degreesToRadians(-sensor.getRate());
  }
}
