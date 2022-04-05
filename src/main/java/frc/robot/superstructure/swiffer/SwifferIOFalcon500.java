// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.misc.exceptions.UnsupportedSubsystemException;
import frc.robot.misc.util.GearingConverter;
import frc.robot.misc.util.sensors.SensorUnitConverter;

public class SwifferIOFalcon500 implements SwifferIO {
  protected final WPI_TalonFX motor;
  protected final GearingConverter gearingConverter;
  protected final boolean isInverted;

  public SwifferIOFalcon500() {
    switch (Constants.getRobot()) {
      case COMP_BOT:
        gearingConverter = new GearingConverter(10);
        motor = new WPI_TalonFX(12);
        isInverted = false;
        break;
      case SIM_BOT:
        gearingConverter = new GearingConverter(25);
        motor = new WPI_TalonFX(8);
        isInverted = false;
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
    inputs.angularVelocityRadiansPerSecond =
        SensorUnitConverter.talonFX.sensorUnitsPer100msToRadiansPerSecond(
            gearingConverter.beforeToAfterGearing(motor.getSelectedSensorVelocity()));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void zeroEncoder() {
    motor.setSelectedSensorPosition(0);
  }

  protected static DCMotor getMotorSim() {
    return DCMotor.getFalcon500(1);
  }
}
