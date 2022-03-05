// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.wheel;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class WheelIOReal implements WheelIO {
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

  public WheelIOReal(Corner corner) {
    switch (frc.robot.Constants.getRobot()) {
      case COMP_BOT:
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
        switch (corner) {
          case FRONT_LEFT:
            motor = new WPI_TalonFX(3);
            break;
          case FRONT_RIGHT:
            motor = new WPI_TalonFX(4);
            motor.setInverted(true);
            break;
          case REAR_LEFT:
            motor = new WPI_TalonFX(5);
            break;
          case REAR_RIGHT:
            motor = new WPI_TalonFX(6);
            motor.setInverted(true);
            break;
          default:
            throw new IllegalArgumentException("Unknown corner");
        }
        break;
      default:
        throw new IllegalStateException("Unknown target robot");
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
  public void setVoltage(double outputVolts) {
    motor.setVoltage(outputVolts);
  }

  @Override
  public void zeroEncoder() {
    motor.setSelectedSensorPosition(0);
  }

  @Override
  public MotorController getMotorController() {
    return motor;
  }
}
