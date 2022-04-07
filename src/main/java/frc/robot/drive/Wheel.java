// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drive.WheelIO.Inputs;
import frc.robot.misc.exceptions.UnknownTargetRobotException;
import frc.robot.misc.util.Clamp;
import frc.robot.misc.util.WheelConverter;
import org.littletonrobotics.junction.Logger;

/** This class should only be used within {@link DriveSubsystem} and {@link Drivebase}. */
class Wheel extends SubsystemBase {
  /** The maximum velocity of a wheel in meters/second. */
  public static final double MAX_WHEEL_VELOCITY;

  /** The maximum acceleration of a wheel in meters/second/second. */
  public static final double MAX_ACCELERATION;

  /** The maximum acceleration of a wheel in radians/second/second. */
  public static final double MAX_ANGULAR_ACCELERATION;

  public static final double MAX_VOLTAGE = 12.0;

  private static final Clamp VOLTAGE_CLAMP = new Clamp(MAX_VOLTAGE);

  private static final SimpleMotorFeedforward FEEDFORWARD;

  private static final WheelConverter WHEEL_CONVERTER;

  static {
    switch (Constants.getRobot()) {
      case TEST_2020_BOT:
        WHEEL_CONVERTER = WheelConverter.fromDiameter(Units.inchesToMeters(5.97));
        MAX_WHEEL_VELOCITY = 4.517538186030606;
        FEEDFORWARD = new SimpleMotorFeedforward(0.088986, 0.18647, 0.0076096);
        break;
      case COMP_BOT:
      case SIM_BOT:
        WHEEL_CONVERTER = WheelConverter.fromDiameter(Units.inchesToMeters(5.97));
        // TODO: Measure the maximum wheel velocity
        MAX_WHEEL_VELOCITY = 4.517538186030606;
        FEEDFORWARD = new SimpleMotorFeedforward(0.060039, 0.22421, 0.011814);
        break;
      default:
        throw new UnknownTargetRobotException();
    }

    // Feedforward is used to convert radians/second to a voltage. You can divide the maximum
    // voltage by the acceleration component of the feedforward to get the robot's average
    // acceleration.
    MAX_ANGULAR_ACCELERATION = VOLTAGE_CLAMP.maximum / FEEDFORWARD.ka;
    MAX_ACCELERATION = WHEEL_CONVERTER.radiansToDistance(MAX_ANGULAR_ACCELERATION);
  }

  /** Wheel velocity PID controller. Input is in radians/second, output is in volts. */
  private final PIDController pid;

  /** This wheel's name in the logger. */
  private final String loggerName;

  private final WheelIO io;
  private final Inputs inputs = new Inputs();

  private double desiredVoltageVolts = 0;

  public Wheel(Corner corner, WheelIO io) {
    this.loggerName = "Wheel/" + corner.toString();
    this.io = io;

    switch (Constants.getRobot()) {
      case TEST_2020_BOT:
        pid = new PIDController(0.031124, 0, 0, Constants.PERIOD_SECONDS);
        break;
      case COMP_BOT:
      case SIM_BOT:
        pid = new PIDController(0.068406, 0, 0, Constants.PERIOD_SECONDS);
        break;
      default:
        throw new UnknownTargetRobotException();
    }

    pid.setSetpoint(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs(loggerName, inputs);
    Logger.getInstance().recordOutput(loggerName + "/VelocityMetersPerSecond", getVelocity());
    Logger.getInstance().recordOutput(loggerName + "/DistanceMeters", getDistance());
    Logger.getInstance()
        .recordOutput(loggerName + "/DesiredVelocityRadiansPerSecond", pid.getSetpoint());
    Logger.getInstance().recordOutput(loggerName + "/DesiredVoltageVolts", desiredVoltageVolts);
  }

  /**
   * Sets the motor output to meet the desired velocity configured with {@link
   * Wheel#setDesiredVelocity(double)}.
   */
  public void doVelocityControlLoop() {
    final var feedforward = FEEDFORWARD.calculate(pid.getSetpoint());
    final var feedback = pid.calculate(inputs.velocityRadiansPerSecond);
    final var voltage = feedforward + feedback;

    desiredVoltageVolts = VOLTAGE_CLAMP.clamp(voltage);

    io.setVoltage(desiredVoltageVolts);
  }

  /**
   * Set desired velocity for the motor.
   *
   * @param metersPerSecond The desired velocity in meters/second
   */
  public void setDesiredVelocity(double metersPerSecond) {
    pid.setSetpoint(WHEEL_CONVERTER.distanceToRadians(metersPerSecond));
  }

  /** Get this wheel's velocity in meters/second. */
  public double getVelocity() {
    return WHEEL_CONVERTER.radiansToDistance(inputs.velocityRadiansPerSecond);
  }

  /** Get the distance in meters this wheel's encoder has travelled since last being reset. */
  public double getDistance() {
    return WHEEL_CONVERTER.radiansToDistance(inputs.positionRadians);
  }
}
