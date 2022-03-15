// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.lifter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.misc.exceptions.UnsupportedSubsystemException;
import frc.robot.misc.io.Falcon500IO;

public class LifterIOFalcon500 extends Falcon500IO implements LifterIO {
  protected final WPI_TalonFX motor;

  protected static final boolean INVERTED = true;

  public LifterIOFalcon500() {
    switch (frc.robot.Constants.getRobot()) {
      case SIM_BOT:
        setGearing(Lifter.GEARING);
        motor = new WPI_TalonFX(1);
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
    inputs.appliedVolts = motor.getMotorOutputVoltage();
    inputs.currentAmps = motor.getSupplyCurrent();
    inputs.tempCelcius = motor.getTemperature();
    // It takes the Phoenix firmware about 4 seconds to boot up in the simulator. During this period
    // all the operations you amek seem to be queued until the motor "is ready". There doesn't
    // appear to be a public API for checking this readiness state, so we just assume that once the
    // driver station is enabled this period has elapsed.
    // TODO: Update this value once I get an answer here:
    // https://www.chiefdelphi.com/t/how-to-check-if-ctre-phoenix-library-has-finished-initialization/405764
    inputs.isReady = DriverStation.isEnabled();
    // Motor is inverted so we invert the encoder measurements as well
    var position = motor.getSelectedSensorPosition();
    var velocity = motor.getSelectedSensorVelocity();

    inputs.positionRadians = sensorUnitsToRadians(position);
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
