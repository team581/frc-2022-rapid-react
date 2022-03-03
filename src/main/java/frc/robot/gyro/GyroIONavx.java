// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C.Port;

public class GyroIONavx implements GyroIO {
  private final AHRS sensor;

  public GyroIONavx() {
    switch (frc.robot.Constants.getRobot()) {
      case TEST_2020_BOT:
      case SIM_BOT:
        sensor = new AHRS(Port.kMXP);
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
    inputs.tempCelcius = sensor.getTempC();
    inputs.rotationRadians = sensor.getRotation2d().getRadians();
    inputs.turnRateRadiansPerSecond = sensor.getRate();
  }
}
