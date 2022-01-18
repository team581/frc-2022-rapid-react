// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Wheel {
  /**
   * The number of arbitrary "native units" per rotation of the sensor.
   *
   * @see https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#talon-fx-integrated-sensor
   */
  private static final int ENCODER_UNITS_PER_ROTATION = 2048;

  final WPI_TalonFX motor;
  final MotorConstants motorConstants;

  final EncoderConstants encoderConstants;

  final SimpleMotorFeedforward feedforward;

  private static final double MOTOR_VOLTAGE = 12;

  public static class MotorConstants {
    public final int port;

    /**
     * The position of the wheel corresponding to this motor, relative to the robot center, in
     * meters.
     */
    public final Translation2d position;

    public MotorConstants(int port, Translation2d position) {
      this.port = port;
      this.position = position;
    }
  }

  public static class EncoderConstants {
    /** The number of rotations of the encoder to travel 1 meter. */
    public final double rotationsPerMeter;

    public EncoderConstants(double rotationsPerMeter) {
      this.rotationsPerMeter = rotationsPerMeter;
    }
  }

  public Wheel(
      MotorConstants motorConstants,
      EncoderConstants encoderConstants,
      SimpleMotorFeedforward feedforward) {
    this.motor = new WPI_TalonFX(motorConstants.port);
    this.motorConstants = motorConstants;

    this.encoderConstants = encoderConstants;

    this.feedforward = feedforward;

    // TODO: Follow the tuning instructions under "Recommended Procedures"
    // https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#changing-velocity-measurement-parameters
    motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    motor.configVelocityMeasurementWindow(1);
  }

  /**
   * Set desired velocity for the motor.
   *
   * @param desiredVelocity The desired velocity in meters/second
   */
  public void setVelocity(double desiredVelocity) {
    // See https://www.chiefdelphi.com/t/falcon-500-closed-loop-velocity/378170/18 for source

    // TalonFX expects velocity over a 100ms period, we use 1s everywhere else in the project though
    double desiredDistancePer100ms = desiredVelocity * Units.millisecondsToSeconds(100);

    motor.set(
        TalonFXControlMode.Velocity,
        desiredDistancePer100ms * encoderConstants.rotationsPerMeter * ENCODER_UNITS_PER_ROTATION,
        DemandType.ArbitraryFeedForward,
        // TODO: Investigate if doing this voltage math is even necessary
        feedforward.calculate(desiredVelocity) / MOTOR_VOLTAGE);
  }
}
