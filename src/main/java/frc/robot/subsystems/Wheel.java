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

  final WheelConstants wheelConstants;

  final EncoderConstants encoderConstants;

  final SimpleMotorFeedforward feedforward;

  private final double kF;

  /**
   * Input: current velocity in meters/second
   *
   * <p>Output: motor voltage as a percentage
   */
  private final PIDController velocityPid;

  public static class MotorConstants {
    public final int port;

    public MotorConstants(int port) {
      this.port = port;
    }
  }

  public static class WheelConstants {

    /**
     * The position of the wheel corresponding to this motor, relative to the robot center, in
     * meters. Used for kinematics.
     */
    public final Translation2d position;

    /** The circumference of the wheel in meters. */
    public final double circumference;

    /**
     * @param position The position of the wheel relative to the robot center, in meters
     * @param diameter The diameter of the wheel in meters
     */
    public WheelConstants(Translation2d position, double diameter) {
      this.position = position;
      this.circumference = diameter * Math.PI;
    }
  }

  public static class EncoderConstants {
    /** The number of rotations of the encoder can do in 1 second at maximum speed. */
    public final int maxEncoderRotationsPerSecond;
    /** The number of encoder rotations for the wheel to rotate once. */
    public final int encoderRotationsPerWheelRotation;

    public EncoderConstants(
        int maxEncoderRotationsPerSecond, int encoderRotationsPerWheelRotation) {
      this.maxEncoderRotationsPerSecond = maxEncoderRotationsPerSecond;
      this.encoderRotationsPerWheelRotation = encoderRotationsPerWheelRotation;
    }
  }

  public Wheel(
      MotorConstants motorConstants,
      EncoderConstants encoderConstants,
      WheelConstants wheelConstants,
      SimpleMotorFeedforward feedforward,
      PIDController velocityPid) {
    this.motor = new WPI_TalonFX(motorConstants.port);
    this.motorConstants = motorConstants;

    this.encoderConstants = encoderConstants;

    this.wheelConstants = wheelConstants;

    this.feedforward = feedforward;
    this.velocityPid = velocityPid;
    velocityPid.setSetpoint(0);

    this.kF = (double) encoderConstants.maxEncoderRotationsPerSecond * ENCODER_UNITS_PER_ROTATION;

    // TODO: These values probably need to be tuned - see tuning instructions
    // https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#recommended-procedure
    motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    motor.configVelocityMeasurementWindow(1);
  }

  /**
   * Sets the motor output to meet the desired velocity configured with {@link
   * Wheel#setDesiredVelocity(double)}.
   */
  public void drive() {
    final var nativePer100ms = motor.getSelectedSensorVelocity();
    final var nativePerSecond = nativePer100ms / Units.millisecondsToSeconds(100);
    final var wheelRotationsPerSecond =
        encoderConstants.encoderRotationsPerWheelRotation * nativePerSecond;
    final var metersPerSecond = wheelRotationsPerSecond * wheelConstants.circumference;

    final var rawVoltage = velocityPid.calculate(metersPerSecond);
    final var clampedVoltage = MathUtil.clamp(rawVoltage, -1, 1);

    motor.set(clampedVoltage);
  }

  /**
   * Set desired velocity for the motor.
   *
   * @param metersPerSecond The desired velocity in meters/second
   */
  public void setDesiredVelocity(double metersPerSecond) {
    velocityPid.setSetpoint(metersPerSecond);
  }
}
