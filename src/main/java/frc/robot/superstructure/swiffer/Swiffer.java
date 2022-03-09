// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.superstructure.swiffer.SwifferIO.Inputs;
import org.littletonrobotics.junction.Logger;

public class Swiffer extends SubsystemBase {
  private static final double MAX_MOTOR_VOLTAGE;
  private static final double TOLERANCE_RPM;
  private static final SimpleMotorFeedforward FEEDFORWARD;

  static {
    switch (Constants.getRobot()) {
      case SIM_BOT:
      default:
        MAX_MOTOR_VOLTAGE = 12;
        TOLERANCE_RPM = 0;
        FEEDFORWARD = new SimpleMotorFeedforward(0, 0, 0);
        break;
    }
  }

  private final PIDController rpmPid;

  private final SwifferIO io;
  private final Inputs inputs = new Inputs();

  private SwifferMode desiredMode;
  private double desiredVoltage = 0;

  /** Creates a new Swiffer. */
  public Swiffer(SwifferIO io) {
    this.io = io;

    switch (Constants.getRobot()) {
      case SIM_BOT:
      default:
        rpmPid = new PIDController(1, 0, 0, Constants.PERIOD_SECONDS);
        break;
    }

    // Flywheel should be stopped when the match starts
    setDesiredMode(SwifferMode.STOPPED);

    rpmPid.setTolerance(TOLERANCE_RPM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Swiffer", inputs);
    Logger.getInstance().recordOutput("Swiffer/DesiredMode", getDesiredMode().toString());
    final var desiredRpm = Units.radiansPerSecondToRotationsPerMinute(getDesiredAngularVelocity());
    Logger.getInstance().recordOutput("Swiffer/DesiredRpm", desiredRpm);
    Logger.getInstance().recordOutput("Swiffer/DesiredAppliedVolts", desiredVoltage);

    doVelocityControlLoop();
  }

  /** Set the desired mode of the flywheel to the one provided. */
  public void setDesiredMode(SwifferMode mode) {
    desiredMode = mode;
    rpmPid.setSetpoint(mode.angularVelocity);
  }

  public boolean atGoal(SwifferMode mode) {
    return mode == getDesiredMode() && rpmPid.atSetpoint();
  }

  private SwifferMode getDesiredMode() {
    return desiredMode;
  }

  /** Get the desired angular velocity in radians/second. */
  private double getDesiredAngularVelocity() {
    return rpmPid.getSetpoint();
  }

  private void doVelocityControlLoop() {
    final var rawVoltage =
        rpmPid.calculate(inputs.angularVelocityRadiansPerSecond)
            + FEEDFORWARD.calculate(getDesiredAngularVelocity());
    final var clampedVoltage = MathUtil.clamp(rawVoltage, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);

    desiredVoltage = clampedVoltage;

    io.setVoltage(clampedVoltage);
  }
}
