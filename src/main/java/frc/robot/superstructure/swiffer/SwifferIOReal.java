// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import frc.robot.Constants;

public class SwifferIOReal implements SwifferIO {
  private static class Constants {
    public static final double SENSOR_RESOLUTION = 2048;
    // 2048 per rotation, so we divide by 1 rotation to get the units per radian
    public static final double SENSOR_UNITS_PER_RADIAN = SENSOR_RESOLUTION / (2 * Math.PI);
  }

  /** Converts sensor units to radians. */
  private static double sensorUnitsToRadians(double sensorUnits) {
    return sensorUnits / Constants.SENSOR_UNITS_PER_RADIAN;
  }

  private final WPI_TalonFX motor;

  public SwifferIOReal() {
    switch (frc.robot.Constants.getRobot()) {
      case SIM_BOT:
        motor = new WPI_TalonFX(2);
        break;
      default:
        throw new IllegalStateException(
            "The currently configured robot doesn't support this subsystem");
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
    inputs.beforeGearingAngularVelocityRadiansPerSecond =
        sensorUnitsToRadians(motor.getSelectedSensorVelocity() * 10);
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
