// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.lifter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import frc.robot.misc.exceptions.UnsupportedSubsystemException;
import frc.robot.misc.io.Falcon500IO;

public class LifterIOReal extends Falcon500IO implements LifterIO {
  private final WPI_TalonFX motor;

  public LifterIOReal() {
    switch (frc.robot.Constants.getRobot()) {
      case SIM_BOT:
        setGearing(1);
        motor = new WPI_TalonFX(1);
        break;
      default:
        throw new UnsupportedSubsystemException(this);
    }

    // TODO: These values probably need to be tuned - see tuning instructions
    // https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#recommended-procedure
    motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    motor.configVelocityMeasurementWindow(1);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.appliedVolts = motor.getMotorOutputVoltage();
    inputs.currentAmps = motor.getSupplyCurrent();
    inputs.tempCelcius = motor.getTemperature();
    inputs.positionRadians = sensorUnitsToRadians(motor.getSelectedSensorPosition());
    inputs.velocityRadiansPerSecond = sensorUnitsToRadians(motor.getSelectedSensorVelocity() * 10);
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
