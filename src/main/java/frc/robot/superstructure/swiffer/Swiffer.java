// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.misc.util.Clamp;
import frc.robot.superstructure.lights.Lights;
import frc.robot.superstructure.swiffer.SwifferIO.Inputs;
import org.littletonrobotics.junction.Logger;

public class Swiffer extends SubsystemBase {
  // TODO: Measure this
  public static final double MASS = 10;

  private static final double TOLERANCE_RPM;
  private static final SimpleMotorFeedforward FEEDFORWARD;
  private static final Clamp VOLTAGE_CLAMP;

  static {
    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        VOLTAGE_CLAMP = new Clamp(12);
        TOLERANCE_RPM = 75;
        // TODO: This needs to be measured again
        FEEDFORWARD = new SimpleMotorFeedforward(0.019184 * 0.6, 0.17836 * 0.6, 0.002161 * 0.6);
        break;
      default:
        VOLTAGE_CLAMP = new Clamp(12);
        TOLERANCE_RPM = 0;
        FEEDFORWARD = new SimpleMotorFeedforward(0, 0, 0);
        break;
    }
  }

  private final PIDController pid;
  private final Lights lights;

  private final SwifferIO io;
  private final Inputs inputs = new Inputs();

  private SwifferMode desiredMode;
  private double desiredVoltageVolts = 0;

  /** Creates a new Swiffer. */
  public Swiffer(SwifferIO io, Lights lights) {
    this.io = io;
    this.lights = lights;

    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        // TODO: This needs to be measured again
        pid = new PIDController(0.0020592 * 0.6, 0, 0, Constants.PERIOD_SECONDS);
        break;
      default:
        pid = new PIDController(1, 0, 0, Constants.PERIOD_SECONDS);
        break;
    }

    // Flywheel should be stopped when the match starts
    setDesiredMode(SwifferMode.STOPPED);

    pid.setTolerance(Units.rotationsPerMinuteToRadiansPerSecond(TOLERANCE_RPM));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Swiffer", inputs);

    doVelocityControlLoop();

    final var atGoal = pid.atSetpoint();
    final var goalRpm = Units.radiansPerSecondToRotationsPerMinute(getDesiredAngularVelocity());
    final var actualRpm =
        Units.radiansPerSecondToRotationsPerMinute(inputs.angularVelocityRadiansPerSecond);
    Logger.getInstance().recordOutput("Swiffer/Goal/Mode", desiredMode.toString());
    Logger.getInstance().recordOutput("Swiffer/Goal/AtGoal", atGoal);
    Logger.getInstance().recordOutput("Swiffer/Goal/Rpm", goalRpm);
    Logger.getInstance().recordOutput("Swiffer/Goal/Error/Rpm", goalRpm - actualRpm);
    Logger.getInstance().recordOutput("Swiffer/Goal/VoltageVolts", desiredVoltageVolts);

    lights.setSubsystemState(desiredMode, atGoal);
  }

  /** Set the desired mode of the flywheel to the one provided. */
  public void setDesiredMode(SwifferMode mode) {
    desiredMode = mode;
    pid.setSetpoint(mode.angularVelocity);
  }

  public boolean atGoal(SwifferMode mode) {
    return mode == desiredMode && pid.atSetpoint();
  }

  /** Get the desired angular velocity in radians/second. */
  private double getDesiredAngularVelocity() {
    return pid.getSetpoint();
  }

  private void doVelocityControlLoop() {
    final var feedforward = FEEDFORWARD.calculate(getDesiredAngularVelocity());
    final var feedback = pid.calculate(inputs.angularVelocityRadiansPerSecond);

    final var voltage = feedback + feedforward;

    desiredVoltageVolts = VOLTAGE_CLAMP.clamp(voltage);

    io.setVoltage(desiredVoltageVolts);
  }
}
