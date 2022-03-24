// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.superstructure.arm.ArmIO.Inputs;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  /** Mass of arm in kilograms. */
  public static final double ARM_MASS = 4776.833 / 1e3;

  /** Length of arm in meters. */
  public static final double ARM_LENGTH = 838.20 / 1e3;

  /** Moment of inertia (Iyy in Fusion 360) in kg m^2. */
  public static final double MOMENT_OF_INERTIA = 3.084;

  /** Gear ratio of motor. */
  public static final double GEARING = 14;

  /** The starting positon of the arm. */
  // Arm starts in the up position
  public static final ArmPosition STARTING_POSITION = ArmPosition.UP;

  private static final double MAX_MOTOR_VOLTAGE;

  private static final TrapezoidProfile.Constraints CONSTRAINTS;

  private static final ArmFeedforward FEEDFORWARD;

  private final ProfiledPIDController pidController;

  static {
    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        // TODO: Use SysID to calculate the feedforward
        FEEDFORWARD = new ArmFeedforward(1, 1.2, 0.4, 0.25);
        MAX_MOTOR_VOLTAGE = 12;
        CONSTRAINTS = new TrapezoidProfile.Constraints(4.6, 28.86);
        break;
      default:
        FEEDFORWARD = new ArmFeedforward(0, 0, 0, 0);
        MAX_MOTOR_VOLTAGE = 12;
        CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
        break;
    }
  }

  private final ArmIO io;
  private final Inputs inputs = new Inputs();

  private ArmPosition desiredPosition = STARTING_POSITION;
  private double desiredVoltage = 0;

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;

    switch (Constants.getRobot()) {
      case SIM_BOT:
      case COMP_BOT:
        // TODO: Use SysID to calculate the PID terms
        pidController =
            new ProfiledPIDController(3.2, 0.35, 0.5, CONSTRAINTS, Constants.PERIOD_SECONDS);
        // TODO: Measure actual acceptable tolerance
        pidController.setTolerance(Units.degreesToRadians(0.01), Units.degreesToRadians(10));
        break;
      default:
        pidController = new ProfiledPIDController(1, 0, 0, CONSTRAINTS, Constants.PERIOD_SECONDS);
        break;
    }

    pidController.setGoal(desiredPosition.state);
    resetController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);

    if (DriverStation.isEnabled()) {
      doPositionControlLoop();
    } else {
      resetController();
    }

    Logger.getInstance().processInputs("Arm", inputs);

    Logger.getInstance().recordOutput("Arm/Goal/Position", desiredPosition.toString());
    Logger.getInstance().recordOutput("Arm/Goal/AtGoal", pidController.atGoal());
    Logger.getInstance().recordOutput("Arm/Goal/PositionRadians", pidController.getGoal().position);
    Logger.getInstance()
        .recordOutput(
            "Arm/Goal/Error/PositionRadians",
            pidController.getGoal().position - inputs.position.getRadians());
    Logger.getInstance()
        .recordOutput(
            "Arm/Goal/Error/VelocityRadiansPerSecond",
            pidController.getGoal().velocity - inputs.velocityRadiansPerSecond);

    Logger.getInstance()
        .recordOutput(
            "Arm/MotionProfile/DesiredPositionRadians", pidController.getSetpoint().position);
    Logger.getInstance()
        .recordOutput(
            "Arm/MotionProfile/DesiredVelocityRadiansPerSecond",
            pidController.getSetpoint().velocity);
    Logger.getInstance().recordOutput("Arm/MotionProfile/DesiredVoltage", desiredVoltage);
    Logger.getInstance()
        .recordOutput("Arm/MotionProfile/Error/PositionRadians", pidController.getPositionError());
    Logger.getInstance()
        .recordOutput(
            "Arm/MotionProfile/Error/VelocityRadiansPerSecond", pidController.getVelocityError());
  }

  private void resetController() {
    pidController.reset(inputs.position.getRadians(), inputs.velocityRadiansPerSecond);
  }

  private void doPositionControlLoop() {
    final var setpoint = pidController.getSetpoint();
    final var feedforward = FEEDFORWARD.calculate(setpoint.position, setpoint.velocity);
    final var feedback = pidController.calculate(inputs.position.getRadians());

    final var voltage = feedforward + feedback;
    desiredVoltage = MathUtil.clamp(voltage, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);

    io.setVoltage(desiredVoltage);
  }

  /** Check if the arm is at the provided position. */
  public boolean atPosition(ArmPosition position) {
    return pidController.getGoal() == position.state && pidController.atGoal();
  }

  /** Set the desired position of the arm. */
  public void setDesiredPosition(ArmPosition position) {
    desiredPosition = position;
    pidController.setGoal(position.state);
  }
}
