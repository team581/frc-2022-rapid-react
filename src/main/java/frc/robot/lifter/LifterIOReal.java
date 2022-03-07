// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lifter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;

public class LifterIOReal implements LifterIO {
  private final WPI_TalonFX motor;

  public LifterIOReal() {
    switch (Constants.getRobot()) {
      case SIM_BOT:
        motor = new WPI_TalonFX(1);
        break;
      default:
        throw new IllegalStateException(
            "The currently configured robot doesn't support this subsystem");
    }
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.appliedVolts = motor.getMotorOutputVoltage();
    inputs.currentAmps = motor.getSupplyCurrent();
    inputs.tempCelcius = motor.getTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void zeroEncoder() {
    motor.setSelectedSensorPosition(0);
  }
}
