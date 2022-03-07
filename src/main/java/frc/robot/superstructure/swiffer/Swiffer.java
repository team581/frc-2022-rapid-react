// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.superstructure.swiffer.SwifferIO.Inputs;
import org.littletonrobotics.junction.Logger;

public class Swiffer implements Subsystem {
  /** The gearing of the flywheel. For example, 10.71:1 would be 10.71. */
  private static final double GEARING;

  private static final double MAX_MOTOR_VOLTAGE;
  private static final double TOLERANCE_RPM;
  private static final SimpleMotorFeedforward FEEDFORWARD;

  static {
    switch (Constants.getRobot()) {
      case SIM_BOT:
        GEARING = 1;
        MAX_MOTOR_VOLTAGE = 12;
        TOLERANCE_RPM = 0;
        FEEDFORWARD = new SimpleMotorFeedforward(0, 0, 0);
        break;
      default:
        throw new IllegalStateException(
            "The currently configured robot doesn't support this subsystem");
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
        rpmPid = new PIDController(1, 0, 0, Constants.PERIOD_SECONDS);
        break;
      default:
        throw new IllegalStateException(
            "The currently configured robot doesn't support this subsystem");
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
    Logger.getInstance().recordOutput("Swiffer/DesiredRpm", getDesiredRpm());
    Logger.getInstance().recordOutput("Swiffer/DesiredAppliedVolts", desiredVoltage);

    doVelocityControlLoop();
  }

  /** Set the desired mode of the flywheel to the one provided. */
  public void setDesiredMode(SwifferMode mode) {
    desiredMode = mode;
    rpmPid.setSetpoint(mode.rpm);
  }

  public boolean atGoal(SwifferMode mode) {
    return mode == getDesiredMode() && rpmPid.atSetpoint();
  }

  private SwifferMode getDesiredMode() {
    return desiredMode;
  }

  private double getDesiredRpm() {
    return rpmPid.getSetpoint();
  }

  private double getRpm() {
    final var angularVelocityRadiansPerSecond =
        inputs.beforeGearingAngularVelocityRadiansPerSecond / GEARING;

    return angularVelocityRadiansPerSecond * 60;
  }

  private void doVelocityControlLoop() {
    final var rawVoltage = rpmPid.calculate(getRpm()) + FEEDFORWARD.calculate(getDesiredRpm());
    final var clampedVoltage = MathUtil.clamp(rawVoltage, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);

    desiredVoltage = clampedVoltage;

    io.setVoltage(clampedVoltage);
  }
}
