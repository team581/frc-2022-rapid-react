// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;

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

  private final double kF;

  private final PIDController pid;

  private final double circumference;

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
    /** The number of rotations of the encoder can do in 1 second at maximum speed. */
    public final double maxEncoderRotationsPerSecond;

    public final double encoderRotationsPerWheelRotation;

    public EncoderConstants(
        double maxEncoderRotationsPerSecond, double encoderRotationsPerWheelRotation) {
      this.maxEncoderRotationsPerSecond = maxEncoderRotationsPerSecond;
      this.encoderRotationsPerWheelRotation = encoderRotationsPerWheelRotation;
    }
  }

  public Wheel(
      MotorConstants motorConstants,
      EncoderConstants encoderConstants,
      SimpleMotorFeedforward feedforward,
      PIDController velocityPid,
      double circumferenceMeters) {
    this.motor = new WPI_TalonFX(motorConstants.port);
    this.motorConstants = motorConstants;

    this.encoderConstants = encoderConstants;

    this.feedforward = feedforward;
    this.pid = velocityPid;

    this.kF = encoderConstants.maxEncoderRotationsPerSecond * ENCODER_UNITS_PER_ROTATION;

    motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    motor.configVelocityMeasurementWindow(1);

    this.circumference = circumferenceMeters;
  }

  public void periodic() {
    final var nativePer100ms = motor.getSelectedSensorVelocity();
    final var nativePerSecond = nativePer100ms * 10;
    final var wheelRotationsPerSecond =
        encoderConstants.encoderRotationsPerWheelRotation * nativePerSecond;
    final var metersPerSecond = wheelRotationsPerSecond * circumference;

    final var raw = pid.calculate(metersPerSecond);
    final var clamped = MathUtil.clamp(raw, -1, 1);

    System.out.println(clamped);

    motor.set(clamped);
  }

  /**
   * Set desired velocity for the motor.
   *
   * @param desiredVelocity The desired velocity in meters/second
   */
  public void setVelocity(double desiredVelocity) {
    pid.setSetpoint(desiredVelocity);
  }
}
