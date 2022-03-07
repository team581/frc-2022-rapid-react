// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.wheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.Drivebase;
import frc.robot.drive.wheel.WheelIO.Inputs;
import org.littletonrobotics.junction.Logger;

/** This class should only be used within {@link DriveSubsystem} and {@link Drivebase}. */
public class Wheel extends SubsystemBase {
  /** The maximum velocity of a wheel in meters/second. */
  public static final double MAX_WHEEL_VELOCITY;

  /** The wheel diameter in meters. */
  private static final double WHEEL_DIAMETER;
  /** The gearing of a wheel. For example, 10.71:1 would be 10.71. */
  private static final double GEARING;

  private static final double MAX_MOTOR_VOLTAGE;

  private static final SimpleMotorFeedforward FEEDFORWARD;

  static {
    switch (Constants.getRobot()) {
      case TEST_2020_BOT:
        GEARING = 10.71;
        WHEEL_DIAMETER = Units.inchesToMeters(5.97);
        MAX_WHEEL_VELOCITY = wheelRotationToMeters(1.011986826 * 2 * Math.PI);
        MAX_MOTOR_VOLTAGE = 12;
        FEEDFORWARD = new SimpleMotorFeedforward(0.61761, 2.3902, 0.17718);
        break;
      case COMP_BOT:
        GEARING = 12.75;
        WHEEL_DIAMETER = Units.inchesToMeters(5.97);
        MAX_WHEEL_VELOCITY = wheelRotationToMeters(2 * Math.PI);
        MAX_MOTOR_VOLTAGE = 12;
        // TODO: Run SysId on the comp bot
        FEEDFORWARD = new SimpleMotorFeedforward(0.61761, 2.3902, 0.17718);
        break;
      case SIM_BOT:
        GEARING = 1;
        WHEEL_DIAMETER = Units.inchesToMeters(5.97);
        MAX_WHEEL_VELOCITY = 1;
        MAX_MOTOR_VOLTAGE = 12;
        FEEDFORWARD = new SimpleMotorFeedforward(0, 0, 0);
        break;
      default:
        throw new IllegalStateException("Unknown target robot");
    }
  }

  /**
   * Converts the wheel's rotation before gearing (in radians) to a distance travelled in meters.
   */
  private static double wheelRotationToMeters(double radians) {
    return radians * (WHEEL_DIAMETER / 2);
  }

  /**
   * The position of the wheel corresponding to this motor, relative to the robot center, in meters.
   * Used for kinematics.
   */
  public final Translation2d positionToCenterOfRobot;

  /**
   * Wheel velocity PID controller.
   *
   * <ul>
   *   <li>Input: current velocity in meters/second.
   *   <li>Output: motor output in volts.
   *   <li>Setpoint: target velocity in meters/second
   * </ul>
   */
  private final PIDController velocityPid;

  /** This wheel's name in the logger. */
  private final String loggerName;

  private final WheelIO io;
  private final Inputs inputs = new Inputs();

  public Wheel(String name, WheelIO io, Translation2d positionToCenterOfRobot) {
    this.loggerName = "Wheel/" + name;
    this.positionToCenterOfRobot = positionToCenterOfRobot;
    this.io = io;

    switch (Constants.getRobot()) {
      case TEST_2020_BOT:
        // TODO: Experiment with different values for max velocity error and max control effort.
        // Remember to check and uncheck "Convert Gains to Encoder Counts" to trigger a refresh and
        // fix the Kp value. Consider rerunning both the quasistatic ramp rate test and the dynamic
        // step voltage test with the robot travelling with 12.0V of energy. This may be unnecessary
        // since the Falcon 500 motors have a pretty linear output of speed:voltage. When recording
        // test results make a note of what voltages were used.
        // Error=1.2m/s MaxControlEffort=7.0V -> Kp = 2.7182 (DEFAULT)
        // Error=0.3m/s MaxControlEffort=12.0V -> Kp = 7.2692
        // Error=0.3m/s MaxControlEffort=7.0V -> Kp = 6.5871
        // Error=0.3m/s MaxControlEffort=7.0V -> Kp = 6.5871
        // Error=0.8m/s MaxControlEffort=7.0V -> Kp = 3.9486
        velocityPid = new PIDController(2.7182, 0, 0, Constants.PERIOD_SECONDS);
        break;
      case COMP_BOT:
        // TODO: Run SysId on the comp bot
        velocityPid = new PIDController(2.7182, 0, 0, Constants.PERIOD_SECONDS);
        break;
      case SIM_BOT:
        velocityPid = new PIDController(1, 0, 0, Constants.PERIOD_SECONDS);
        break;
      default:
        throw new IllegalStateException("Unknown target robot");
    }

    velocityPid.setSetpoint(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs(loggerName, inputs);
    Logger.getInstance().recordOutput(loggerName + "/VelocityMetersPerSecond", getVelocity());
    Logger.getInstance().recordOutput(loggerName + "/DistanceMeters", getDistance());
    Logger.getInstance()
        .recordOutput(loggerName + "/DesiredVelocityMetersPerSecond", getDesiredVelocity());
    Logger.getInstance().recordOutput(loggerName + "/DesiredAppliedVolts", getDesiredVoltage());
  }

  /**
   * Sets the motor output to meet the desired velocity configured with {@link
   * Wheel#setDesiredVelocity(double)}.
   */
  public void doVelocityControlLoop() {
    io.setVoltage(getDesiredVoltage());
  }

  /** Get the desired motor voltage. */
  private double getDesiredVoltage() {
    return velocityToVoltage(getDesiredVelocity());
  }

  /** Get the desired velocity in meters/second. */
  private double getDesiredVelocity() {
    return velocityPid.getSetpoint();
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
    final var radiansPerSecond = inputs.beforeGearingVelocityRadiansPerSecond / GEARING;

    return wheelRotationToMeters(radiansPerSecond);
  }

  /** Get the distance in meters this wheel's encoder has travelled since last being reset. */
  public double getDistance() {
    final var radians = inputs.beforeGearingPositionRadians / GEARING;

    return wheelRotationToMeters(radians);
  }

  /** Zeroes the encoder position. */
  public void zeroEncoder() {
    io.zeroEncoder();
  }

  /** Converts a velocity in meters/second to a voltage. */
  private double velocityToVoltage(double velocity) {
    final var rawVoltage = FEEDFORWARD.calculate(velocity) + velocityPid.calculate(velocity);

    return MathUtil.clamp(rawVoltage, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);
  }
}
