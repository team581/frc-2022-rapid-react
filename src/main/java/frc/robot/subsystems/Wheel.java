// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;

public class Wheel {
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
  }

  /**
   * Set desired velocity for the motor.
   *
   * @param desiredVelocity The desired velocity in meters/second
   */
  public void setVelocity(double desiredVelocity) {
    // See https://www.chiefdelphi.com/t/falcon-500-closed-loop-velocity/378170/18 for source

    // TalonFX expects velocity over a 100ms period, we use 1s everywhere else in the project though
    double desiredVelocityPer100ms = desiredVelocity * 0.1;

    motor.set(
        TalonFXControlMode.Velocity,
        desiredVelocityPer100ms * encoderConstants.rotationsPerMeter * 2048,
        DemandType.ArbitraryFeedForward,
        feedforward.calculate(desiredVelocity) / MOTOR_VOLTAGE);
  }
}
