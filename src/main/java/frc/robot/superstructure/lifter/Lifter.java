// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.lifter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.superstructure.lifter.LifterIO.Inputs;
import org.littletonrobotics.junction.Logger;

public class Lifter extends SubsystemBase {
  private static final double MAX_MOTOR_VOLTAGE;

  /** The maximum acceptable error in position (radians). */
  private static final double POSITION_TOLERANCE;

  private static final double VELOCITY_TOLERANCE;

  private static final ArmFeedforward FEEDFORWARD;
  private static final TrapezoidProfile.Constraints CONSTRAINTS;

  static {
    switch (Constants.getRobot()) {
      case SIM_BOT:
      default:
        MAX_MOTOR_VOLTAGE = 12;
        POSITION_TOLERANCE = Rotation2d.fromDegrees(0).getRadians();
        VELOCITY_TOLERANCE = 0;
        FEEDFORWARD = new ArmFeedforward(0.05, 0.5, 0);
        CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
        break;
    }
  }

  private final ProfiledPIDController positionPid;

  private final LifterIO io;
  private final Inputs inputs = new Inputs();

  private LifterPosition desiredPosition;

  /** Creates a new Lifter. */
  public Lifter(LifterIO io) {
    this.io = io;

    switch (Constants.getRobot()) {
      case SIM_BOT:
      default:
        positionPid = new ProfiledPIDController(1.2, 0, 0, CONSTRAINTS, Constants.PERIOD_SECONDS);
        break;
    }

    positionPid.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);

    // Lifter starts in the up position at match start
    setDesiredPosition(LifterPosition.UP);
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
    Logger.getInstance()
        .recordOutput("Lifter/MotionProfiledPid/DesiredVelocityMetersPerSecond", state.velocity);
    Logger.getInstance()
        .recordOutput("Lifter/MotionProfiledPid/DesiredPositionRadians", state.position);

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
  public void setDesiredPosition(LifterPosition position) {
    desiredPosition = position;
    setDesiredPosition(position.angle);
  }

  /** Check if the lifter is at the provided position. */
  public boolean atPosition(LifterPosition position) {
    return desiredPosition == position && positionPid.atGoal();
  }

  /** Set the desired position of the lifter to the provided angle. */
  private void setDesiredPosition(Rotation2d angle) {
    positionPid.setGoal(angle.getRadians());
  }

  /** Gets the actual position of the lifter. */
  private Rotation2d getPosition() {
    return new Rotation2d(inputs.positionRadians);
  }
}
