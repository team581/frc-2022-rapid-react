// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import frc.robot.Constants;
import frc.robot.misc.exceptions.UnknownTargetRobotException;
import frc.robot.misc.util.GearingConverter;
import frc.robot.misc.util.sensors.SensorUnitConverter;

public class WheelIOFalcon500 implements WheelIO {
  private final WPI_TalonFX motor;
  private final GearingConverter gearingConverter;

  public WheelIOFalcon500(Corner corner) {
    switch (Constants.getRobot()) {
      case COMP_BOT:
        gearingConverter = new GearingConverter(12.75);
        switch (corner) {
          case FRONT_LEFT:
            motor = new WPI_TalonFX(14);
            break;
          case FRONT_RIGHT:
            motor = new WPI_TalonFX(11);
            motor.setInverted(true);
            break;
          case REAR_LEFT:
            motor = new WPI_TalonFX(12);
            break;
          case REAR_RIGHT:
            motor = new WPI_TalonFX(13);
            motor.setInverted(true);
            break;
          default:
            throw new IllegalArgumentException("Unknown corner");
        }
        break;
      case TEST_2020_BOT:
        gearingConverter = new GearingConverter(10.71);
        switch (corner) {
          case FRONT_LEFT:
            motor = new WPI_TalonFX(10);
            break;
          case FRONT_RIGHT:
            motor = new WPI_TalonFX(11);
            motor.setInverted(true);
            break;
          case REAR_LEFT:
            motor = new WPI_TalonFX(12);
            break;
          case REAR_RIGHT:
            motor = new WPI_TalonFX(13);
            motor.setInverted(true);
            break;
          default:
            throw new IllegalArgumentException("Unknown corner");
        }
        break;
      case SIM_BOT:
        gearingConverter = new GearingConverter(1);
        switch (corner) {
          case FRONT_LEFT:
            motor = new WPI_TalonFX(1);
            break;
          case FRONT_RIGHT:
            motor = new WPI_TalonFX(2);
            motor.setInverted(true);
            break;
          case REAR_LEFT:
            motor = new WPI_TalonFX(3);
            break;
          case REAR_RIGHT:
            motor = new WPI_TalonFX(4);
            motor.setInverted(true);
            break;
          default:
            throw new IllegalArgumentException("Unknown corner");
        }
        break;
      default:
        throw new UnknownTargetRobotException();
    }
    motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    motor.configVelocityMeasurementWindow(1);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.voltageVolts = motor.getMotorOutputVoltage();
    inputs.currentAmps = motor.getSupplyCurrent();
    inputs.tempCelcius = motor.getTemperature();
    inputs.positionRadians =
        SensorUnitConverter.talonFX.sensorUnitsToRadians(
            gearingConverter.beforeToAfterGearing(motor.getSelectedSensorPosition()));
    inputs.velocityRadiansPerSecond =
        SensorUnitConverter.talonFX.sensorUnitsPer100msToRadiansPerSecond(
            gearingConverter.beforeToAfterGearing(motor.getSelectedSensorVelocity()));
  }

  @Override
  public void setVoltage(double outputVolts) {
    motor.setVoltage(outputVolts);
  }

  @Override
  public void zeroEncoder() {
    motor.setSelectedSensorPosition(0);
  }
}
