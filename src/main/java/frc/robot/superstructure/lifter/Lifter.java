// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.lifter;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.superstructure.lifter.LifterIO.Inputs;
import org.littletonrobotics.junction.Logger;

public class Lifter extends SubsystemBase {
  /** Mass of arm in kilograms. */
  // TODO: Update from CAD once arm is finalized
  public static final double ARM_MASS = 5;

  /** Length of arm in meters. */
  // TODO: Update from CAD once arm is finalized
  public static final double ARM_LENGTH = 2.076;

  /** Gear ratio of motor. */
  public static final double GEARING = 20;

  private static final double MAX_MOTOR_VOLTAGE;

  private static final TrapezoidProfile.Constraints CONSTRAINTS;

  // In this example we weight position much more highly than velocity, but this can be tuned to
  // balance the two.
  /** Maximum acceptable position error (in radians). */
  private static final double MAX_POSITION_ERROR = Units.degreesToRadians(1);

  /** Maximum acceptable velocity error (in radians per second). */
  private static final double MAX_VELOCITY_ERROR = Units.degreesToRadians(10);

  static {
    switch (Constants.getRobot()) {
      case SIM_BOT:
      default:
        MAX_MOTOR_VOLTAGE = 12;
        CONSTRAINTS = new TrapezoidProfile.Constraints(2, 3);
        break;
    }
  }

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N2, N1, N1> loop;

  private final LifterIO io;
  private final Inputs inputs = new Inputs();

  // Lifter starts in the up position
  private LifterPosition desiredPosition = LifterPosition.UP;
  private TrapezoidProfile.State lastProfiledReference = desiredPosition.state;

  /** Creates a new Lifter. */
  public Lifter(LifterIO io) {
    this.io = io;

    final LinearSystem<N2, N1, N1> armPlant =
        LinearSystemId.createSingleJointedArmSystem(
            io.getMotorSim(), SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS), GEARING);

    final KalmanFilter<N2, N1, N1> observer =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            armPlant,
            // How accurate we think our model is, in radians and radians/sec
            VecBuilder.fill(0.015, 0.17),
            // How accurate we think our encoder position data is. In this case we very highly trust
            // our encoder position reading.
            VecBuilder.fill(0.01),
            Constants.PERIOD_SECONDS);

    // A LQR uses feedback to create voltage commands.
    final LinearQuadraticRegulator<N2, N1, N1> controller =
        new LinearQuadraticRegulator<>(
            armPlant,
            // qelms.
            // Position and velocity error tolerances, in radians and radians per second. Decrease
            // this to more heavily penalize state excursion, or make the controller behave more
            // aggressively.
            VecBuilder.fill(MAX_POSITION_ERROR, MAX_VELOCITY_ERROR),
            // relms. Control effort (voltage) tolerance. Decrease this to more heavily penalize
            // control effort, or make the controller less aggressive. 12 is a good starting point
            // because that is the (approximate) maximum voltage of a battery.
            VecBuilder.fill(MAX_MOTOR_VOLTAGE),
            Constants.PERIOD_SECONDS);

    loop =
        new LinearSystemLoop<>(
            armPlant, controller, observer, MAX_MOTOR_VOLTAGE, Constants.PERIOD_SECONDS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);

    Logger.getInstance().processInputs("Lifter", inputs);

    Logger.getInstance().recordOutput("Lifter/DesiredPosition", desiredPosition.toString());
    Logger.getInstance()
        .recordOutput("Lifter/DesiredPositionRadians", desiredPosition.state.position);
    Logger.getInstance()
        .recordOutput("Lifter/PositionConstants/Up", LifterPosition.UP.state.position);
    Logger.getInstance()
        .recordOutput("Lifter/PositionConstants/Down", LifterPosition.DOWN.state.position);

    if (atPosition(LifterPosition.UP)) {
      Logger.getInstance().recordOutput("Lifter/Position", LifterPosition.UP.toString());
    } else if (atPosition(LifterPosition.DOWN)) {
      Logger.getInstance().recordOutput("Lifter/Position", LifterPosition.DOWN.toString());
    } else {
      Logger.getInstance().recordOutput("Lifter/Position", "UNKNOWN");
    }

    doPositionControlLoop();

    Logger.getInstance()
        .recordOutput("Lifter/Reference/DesiredPositionRadians", lastProfiledReference.position);
    Logger.getInstance()
        .recordOutput(
            "Lifter/Reference/DesiredVelocityRadiansPerSecond", lastProfiledReference.velocity);
  }

  private void doPositionControlLoop() {
    lastProfiledReference =
        new TrapezoidProfile(CONSTRAINTS, desiredPosition.state, lastProfiledReference)
            .calculate(Constants.PERIOD_SECONDS);
    loop.setNextR(lastProfiledReference.position, lastProfiledReference.velocity);

    // Correct our Kalman filter's state vector estimate with encoder data
    loop.correct(VecBuilder.fill(inputs.positionRadians));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter
    loop.predict(Constants.PERIOD_SECONDS);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    final var nextVoltage = loop.getU(0);
    io.setVoltage(nextVoltage);
  }

  /** Check if the lifter is at the provided position. */
  public boolean atPosition(LifterPosition position) {
    if (desiredPosition != position) {
      // Wrong position
      return false;
    }

    final var angleError = loop.getError(0);
    if (Math.abs(angleError) > MAX_POSITION_ERROR) {
      // Angle error tolerance exceeded
      return false;
    }

    final var angularVelocityError = loop.getError(1);
    // Velocity error tolerance exceeded
    return Math.abs(angularVelocityError) < MAX_VELOCITY_ERROR;
  }

  /** Set the desired position of the lifter. */
  public void setDesiredPosition(LifterPosition position) {
    desiredPosition = position;
  }
}
