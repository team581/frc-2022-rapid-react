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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Wheel {
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
    public final double maxEncoderRotationsPerSecond;
    /** The number of encoder rotations for the wheel to rotate once. */
    public final double encoderRotationsPerWheelRotation;

    public EncoderConstants(
        double maxEncoderRotationsPerSecond, double encoderRotationsPerWheelRotation) {
      this.maxEncoderRotationsPerSecond = maxEncoderRotationsPerSecond;
      this.encoderRotationsPerWheelRotation = encoderRotationsPerWheelRotation;
    }
  }

  public Wheel(
      String name,
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

    this.kF = encoderConstants.maxEncoderRotationsPerSecond;

    // TODO: These values probably need to be tuned - see tuning instructions
    // https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#recommended-procedure
    motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    motor.configVelocityMeasurementWindow(1);

    Shuffleboard.getTab("Robot").addNumber("velocity/" + name, () -> this.getVelocity());
    Shuffleboard.getTab("Robot").addNumber("distance/" + name, () -> this.getDistance());
  }

  /**
   * Sets the motor output to meet the desired velocity configured with {@link
   * Wheel#setDesiredVelocity(double)}.
   */
  public void drive() {
    final var rawVoltage = velocityPid.calculate(getVelocity());
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

  /** Get this wheel's velocity in meters/second. */
  public double getVelocity() {
    // This is definitely per 100ms
    final var nativePer100Ms = motor.getSelectedSensorVelocity();
    final var nativePerSecond = nativePer100Ms * 10;

    return encoderRotationsToMeters(nativePerSecond);
  }

  /** Get the distance in meters this wheel's encoder has travelled since last being reset. */
  public double getDistance() {
    final var nativeDistance = motor.getSelectedSensorPosition();

    return encoderRotationsToMeters(nativeDistance);
  }

  /** Convert encoder rotations to meters this wheel has travelled since last being reset. */
  private double encoderRotationsToMeters(double encoderRotations) {
    final var wheelRotations = encoderRotations / encoderConstants.encoderRotationsPerWheelRotation;

    return wheelRotations * wheelConstants.circumference;
  }

  public void resetEncoder() {
    motor.setSelectedSensorPosition(0);
  }
}
