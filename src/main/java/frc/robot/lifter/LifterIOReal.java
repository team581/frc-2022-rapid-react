// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lifter;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class LifterIOReal implements LifterIO {
  private final TalonFX motor;
  private final DigitalInput upperLimitSwitch;
  private final DigitalInput lowerLimitSwitch;

  public LifterIOReal() {
    switch (Constants.getRobot()) {
      case SIM_BOT:
        motor = new TalonFX(0);
        upperLimitSwitch = new DigitalInput(0);
        lowerLimitSwitch = new DigitalInput(1);
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

    inputs.upperLimitSwitchActive = upperLimitSwitch.get();

    inputs.lowerLimitSwitchActive = lowerLimitSwitch.get();
  }

  @Override
  public void setMotorPercentage(double percentage) {
    motor.set(TalonFXControlMode.PercentOutput, percentage);
  }
}
