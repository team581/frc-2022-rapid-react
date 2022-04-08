// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.misc.util.Clamp;
import frc.robot.superstructure.arm.ArmIO.Inputs;
import frc.robot.superstructure.lights.Lights;
import frc.robot.superstructure.swiffer.Swiffer;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  /** Mass of arm in kilograms. */
  public static final double ARM_MASS = Swiffer.MASS + (4776.833 / 1e3);

  /** Length of arm in meters. */
  public static final double ARM_LENGTH = 838.20 / 1e3;

  /** Moment of inertia (Iyy in Fusion 360) in kg m^2. */
  public static final double MOMENT_OF_INERTIA = 3.084;

  /** Gear ratio of motor. */
  public static final double GEARING = 300.0 / 7.0;

  /** The starting positon of the arm. */
  // Arm starts in the up position
  public static final ArmPosition STARTING_POSITION = ArmPosition.UP;

  private static final Clamp VOLTAGE_CLAMP;

  private static final TrapezoidProfile.Constraints CONSTRAINTS;

  private static final ArmFeedforward FEEDFORWARD;

  static {
    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        // TODO: Use SysID to calculate the feedforward
        FEEDFORWARD = new ArmFeedforward(1.2, 0.95, 2, 0.08);
        VOLTAGE_CLAMP = new Clamp(12);
        CONSTRAINTS =
            // We use the actual max acceleration here but limit the velocity to avoid breaking the
            // arm or tipping the robot
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(20), VOLTAGE_CLAMP.maximum / FEEDFORWARD.ka);
        break;
      default:
        FEEDFORWARD = new ArmFeedforward(0, 0, 0, 0);
        VOLTAGE_CLAMP = new Clamp(12);
        CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
        break;
    }
  }

  private final ProfiledPIDController pidController;
  private final Lights lights;

  private final ArmIO io;
  private final Inputs inputs = new Inputs();

  private ArmPosition desiredPosition = STARTING_POSITION;
  private double desiredVoltageVolts = 0;

  /** Creates a new Arm. */
  public Arm(ArmIO io, Lights lights) {
    this.io = io;
    this.lights = lights;

    switch (Constants.getRobot()) {
      case SIM_BOT:
      case COMP_BOT:
        // TODO: Use SysID to calculate the PID terms
        pidController =
            new ProfiledPIDController(1.5, 0, 0.4, CONSTRAINTS, Constants.PERIOD_SECONDS);
        // TODO: Measure actual acceptable tolerance
        pidController.setTolerance(Units.degreesToRadians(4), Units.degreesToRadians(8));
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

    final var atGoal = pidController.atGoal();
    Logger.getInstance().recordOutput("Arm/Goal/Position", desiredPosition.toString());
    Logger.getInstance().recordOutput("Arm/Goal/AtGoal", atGoal);
    Logger.getInstance().recordOutput("Arm/Goal/PositionRadians", pidController.getGoal().position);
    Logger.getInstance()
        .recordOutput(
            "Arm/Goal/Error/PositionRadians",
            pidController.getGoal().position - inputs.positionRadians);
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
    Logger.getInstance().recordOutput("Arm/MotionProfile/DesiredVoltageVolts", desiredVoltageVolts);
    Logger.getInstance()
        .recordOutput("Arm/MotionProfile/Error/PositionRadians", pidController.getPositionError());
    Logger.getInstance()
        .recordOutput(
            "Arm/MotionProfile/Error/VelocityRadiansPerSecond", pidController.getVelocityError());

    lights.setSubsystemState(desiredPosition, atGoal);
  }

  private void resetController() {
    pidController.reset(inputs.positionRadians, inputs.velocityRadiansPerSecond);
  }

  private void doPositionControlLoop() {
    final var setpoint = pidController.getSetpoint();
    final var feedforward = FEEDFORWARD.calculate(setpoint.position, setpoint.velocity);
    final var feedback = pidController.calculate(inputs.positionRadians);

    final var voltage = feedforward + feedback;
    desiredVoltageVolts = VOLTAGE_CLAMP.clamp(voltage);

    io.setVoltage(desiredVoltageVolts);
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
