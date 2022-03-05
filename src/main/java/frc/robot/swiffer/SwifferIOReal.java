// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swiffer;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;

public class SwifferIOReal implements SwifferIO {
  private final TalonFX motor;

  public SwifferIOReal() {
    switch (Constants.getRobot()) {
      case SIM_BOT:
        motor = new TalonFX(2);
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
  public void setMotorPercentage(double percentage) {
    motor.set(TalonFXControlMode.PercentOutput, percentage);
  }
}
