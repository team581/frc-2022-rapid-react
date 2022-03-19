// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.misc.exceptions.UnsupportedSubsystemException;
import frc.robot.misc.io.Falcon500IO;

public class ArmIOFalcon500 extends Falcon500IO implements ArmIO {
  protected final WPI_TalonFX motor;

  protected static final boolean INVERTED = true;

  public ArmIOFalcon500() {
    switch (Constants.getRobot()) {
      case SIM_BOT:
        setGearing(Arm.GEARING);
        motor = new WPI_TalonFX(5);
        break;
      default:
        throw new UnsupportedSubsystemException(this);
    }

    motor.setInverted(INVERTED);

    // TODO: These values probably need to be tuned - see tuning instructions
    // https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#recommended-procedure
    motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    motor.configVelocityMeasurementWindow(1);
  }

  @Override
  public DCMotor getMotorSim() {
    return DCMotor.getFalcon500(1);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.appliedVolts = new double[] {motor.getMotorOutputVoltage()};
    inputs.currentAmps = new double[] {motor.getSupplyCurrent()};
    inputs.tempCelcius = new double[] {motor.getTemperature()};
    var position = motor.getSelectedSensorPosition();
    var velocity = motor.getSelectedSensorVelocity();

    inputs.position = new Rotation2d(sensorUnitsToRadians(position));
    inputs.velocityRadiansPerSecond = sensorUnitsPer100msToRadiansPerSecond(velocity);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setEncoderPosition(Rotation2d rotation) {
    motor.setSelectedSensorPosition(radiansToSensorUnits(rotation.getRadians()));
  }
}