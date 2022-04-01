// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.imu;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Constants;
import frc.robot.misc.exceptions.UnsupportedSubsystemException;

public class ImuIONavx implements ImuIO {
  private final AHRS sensor;

  public ImuIONavx() {
    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        sensor = new AHRS(Port.kMXP);
        break;
      default:
        throw new UnsupportedSubsystemException(this);
    }
  }

  @Override
  public void zeroHeading() {
    sensor.reset();
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.tempCelcius = sensor.getTempC();
    inputs.rotationRadians = Units.degreesToRadians(-sensor.getAngle());
    inputs.turnRateRadiansPerSecond = Units.degreesToRadians(-sensor.getRate());
  }
}
