// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lifter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lifter.LifterIO.Inputs;
import org.littletonrobotics.junction.Logger;

public class LifterSubsystem extends SubsystemBase {

  /** The gearing of a wheel. For example, 10.71:1 would be 10.71. */
  private static final double GEARING;

  private static final double MAX_MOTOR_VOLTAGE;
  private static final ArmFeedforward FEEDFORWARD;
  private static final TrapezoidProfile.Constraints CONSTRAINTS;

  static {
    switch (Constants.getRobot()) {
      case SIM_BOT:
        GEARING = 1;
        MAX_MOTOR_VOLTAGE = 12;
        FEEDFORWARD = new ArmFeedforward(0, 0, 0);
        CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
        break;
      default:
        throw new IllegalStateException(
            "The currently configured robot doesn't support this subsystem");
    }
  }

  private final ProfiledPIDController positionPid;

  private final LifterIO io;
  private final Inputs inputs = new Inputs();

  private Position desiredPosition = Position.DOWN;

  /** Creates a new LifterSubsystem. */
  public LifterSubsystem(LifterIO io) {
    this.io = io;

    switch (Constants.getRobot()) {
      case SIM_BOT:
        positionPid = new ProfiledPIDController(1, 0, 0, CONSTRAINTS);
        break;
      default:
        throw new IllegalStateException(
            "The currently configured robot doesn't support this subsystem");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Lifter", inputs);
    Logger.getInstance().recordOutput("Lifter/DesiredPosition", desiredPosition.toString());
    Logger.getInstance()
        .recordOutput("Lifter/DesiredPositionRadians", positionPid.getGoal().position);

    final var state = positionPid.getSetpoint();
    Logger.getInstance().recordOutput("Lifter/MotionProfile/DesiredVelocityMeters", state.velocity);
    Logger.getInstance()
        .recordOutput("Lifter/MotionProfile/DesiredPositionRadians", state.position);

    doPositionControlLoop();
  }

  private void doPositionControlLoop() {
    final var state = positionPid.getSetpoint();

    final var rawVoltage =
        positionPid.calculate(getPosition().getRadians())
            + FEEDFORWARD.calculate(state.position, state.velocity);

    final var clampedVoltage = MathUtil.clamp(rawVoltage, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);

    io.setVoltage(clampedVoltage);
  }

  /** Set the desired position of the lifter to the provided position. */
  public void setDesiredPosition(Position position) {
    desiredPosition = position;
    setDesiredPosition(position.angle);
  }

  /** Check if the lifter is at the provided position. */
  public boolean atPosition(Position position) {
    return desiredPosition == position && positionPid.atGoal();
  }

  /** Set the desired position of the lifter to the provided angle. */
  private void setDesiredPosition(Rotation2d angle) {
    positionPid.setGoal(angle.getRadians());
  }

  /** Gets the actual position of the lifter. */
  private Rotation2d getPosition() {
    return new Rotation2d(inputs.beforeGearingPositionRadians / GEARING);
  }
}
