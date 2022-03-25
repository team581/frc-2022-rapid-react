// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.wheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.Drivebase;
import frc.robot.drive.wheel.WheelIO.Inputs;
import frc.robot.misc.exceptions.UnknownTargetRobotException;
import frc.robot.misc.util.Clamp;
import frc.robot.misc.util.WheelConverter;
import org.littletonrobotics.junction.Logger;

/** This class should only be used within {@link DriveSubsystem} and {@link Drivebase}. */
public class Wheel extends SubsystemBase {
  /** The maximum velocity of a wheel in meters/second. */
  public static final double MAX_WHEEL_VELOCITY;

  private static final Clamp VOLTAGE_CLAMP;

  private static final SimpleMotorFeedforward FEEDFORWARD;

  private static final WheelConverter WHEEL_CONVERTER;

  static {
    switch (Constants.getRobot()) {
      case TEST_2020_BOT:
        WHEEL_CONVERTER = WheelConverter.fromDiameter(Units.inchesToMeters(5.97));
        MAX_WHEEL_VELOCITY =
            WHEEL_CONVERTER.radiansToDistance(Units.rotationsToRadians(1.011986826));
        VOLTAGE_CLAMP = new Clamp(12);
        FEEDFORWARD = new SimpleMotorFeedforward(0.61761, 2.3902, 0.17718);
        break;
      case COMP_BOT:
        WHEEL_CONVERTER = WheelConverter.fromDiameter(Units.inchesToMeters(5.97));
        // TODO: Measure the maximum wheel velocity
        MAX_WHEEL_VELOCITY = WHEEL_CONVERTER.radiansToDistance(Units.rotationsToRadians(1));
        VOLTAGE_CLAMP = new Clamp(12);
        // TODO: Run SysId on the comp bot
        FEEDFORWARD = new SimpleMotorFeedforward(0.61761, 2.3902, 0.17718);
        break;
      case SIM_BOT:
        WHEEL_CONVERTER = WheelConverter.fromDiameter(Units.inchesToMeters(5.97));
        MAX_WHEEL_VELOCITY = 1;
        VOLTAGE_CLAMP = new Clamp(12);
        FEEDFORWARD = new SimpleMotorFeedforward(0, 0, 0);
        break;
      default:
        throw new UnknownTargetRobotException();
    }
  }

  /**
   * The position of the wheel corresponding to this motor, relative to the robot center, in meters.
   * Used for kinematics.
   */
  public final Translation2d positionToCenterOfRobot;

  /** Wheel velocity PID controller. Input is in radians/second, output is in volts. */
  private final PIDController pid;

  /** This wheel's name in the logger. */
  private final String loggerName;

  private final WheelIO io;
  private final Inputs inputs = new Inputs();

  private double desiredVoltageVolts = 0;

  public Wheel(Corner corner, WheelIO io, Translation2d positionToCenterOfRobot) {
    this.loggerName = "Wheel/" + corner.toString();
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
        // TODO: This needs to be recalculated to use radians instead of meters. So do the example
        // values shown above.
        pid = new PIDController(2.7182, 0, 0, Constants.PERIOD_SECONDS);
        break;
      case COMP_BOT:
        // TODO: Run SysId on the comp bot
        pid = new PIDController(2.7182, 0, 0, Constants.PERIOD_SECONDS);
        break;
      case SIM_BOT:
        pid = new PIDController(1, 0, 0, Constants.PERIOD_SECONDS);
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
    pid.setSetpoint(WHEEL_CONVERTER.velocityToAngularVelocity(metersPerSecond));
  }

  /** Get this wheel's velocity in meters/second. */
  public double getVelocity() {
    return WHEEL_CONVERTER.angularVelocityToVelocity(inputs.velocityRadiansPerSecond);
  }

  /** Get the distance in meters this wheel's encoder has travelled since last being reset. */
  public double getDistance() {
    return WHEEL_CONVERTER.radiansToDistance(inputs.positionRadians);
  }

  /** Zeroes the encoder position. */
  public void zeroEncoder() {
    io.zeroEncoder();
  }
}
