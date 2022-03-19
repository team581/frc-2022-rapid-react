// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.lifter;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.superstructure.lifter.LifterIO.Inputs;
import org.littletonrobotics.junction.Logger;

public class Lifter extends SubsystemBase {
  /** Mass of arm in kilograms. */
  public static final double ARM_MASS = 4776.833 / 1e3;

  /** Length of arm in meters. */
  public static final double ARM_LENGTH = 838.20 / 1e3;

  /** Moment of inertia (Iyy in Fusion 360) in kg m^2. */
  public static final double MOMENT_OF_INERTIA = 3.084;

  /** Gear ratio of motor. */
  public static final double GEARING = 14;

  private static final double MAX_MOTOR_VOLTAGE;

  private static final TrapezoidProfile.Constraints CONSTRAINTS;

  // In this example we weight position much more highly than velocity, but this can be tuned to
  // balance the two.
  /** Maximum acceptable position error (in radians). */
  private static final double MAX_POSITION_ERROR = Units.degreesToRadians(2);

  /** Maximum acceptable angular velocity error (in radians per second). */
  private static final double MAX_VELOCITY_ERROR = Units.degreesToRadians(3);

  /**
   * A feedforward for the arm's gravity. An entire {@link ArmFeedforward} instance isn't required
   * as we're setting every value to 0 except for the gravity gain (kcos), you could just add <code>
   * kcos * cos(desiredAngle)</code>. Using an entire feedforward object is clearer though.
   */
  private static final ArmFeedforward GRAVITY_FEEDFORWARD;

  static {
    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        // TODO: Use SysID to calculate the gravity term
        GRAVITY_FEEDFORWARD = new ArmFeedforward(0, 0, 0, 0);
        MAX_MOTOR_VOLTAGE = 12;
        CONSTRAINTS = new TrapezoidProfile.Constraints(4.6, 28.86);
        break;
      default:
        GRAVITY_FEEDFORWARD = new ArmFeedforward(0, 0, 0, 0);
        MAX_MOTOR_VOLTAGE = 12;
        CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
        break;
    }
  }

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N2, N1, N1> loop;

  private final LifterIO io;
  private final Inputs inputs = new Inputs();

  // Lifter starts in the up position
  private LifterPosition desiredPosition = LifterPosition.UP;
  private TrapezoidProfile.State lastProfiledReference;
  private double nextVoltage = 0;

  /** Creates a new Lifter. */
  public Lifter(LifterIO io) {
    this.io = io;

    seedSensorPosition();

    final LinearSystem<N2, N1, N1> armPlant =
        LinearSystemId.createSingleJointedArmSystem(io.getMotorSim(), MOMENT_OF_INERTIA, GEARING);

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
    Logger.getInstance().recordOutput("Lifter/AtReference", atPosition(desiredPosition));

    // Avoid messing up the Kalman filter's state by making it believe we're using its output
    // voltages when the robot is disabled
    if (DriverStation.isEnabled()) {
      doPositionControlLoop();
    }

    Logger.getInstance()
        .recordOutput("Lifter/Reference/DesiredPositionRadians", lastProfiledReference.position);
    Logger.getInstance()
        .recordOutput(
            "Lifter/Reference/DesiredVelocityRadiansPerSecond", lastProfiledReference.velocity);
    Logger.getInstance().recordOutput("Lifter/DesiredAppliedVolts", nextVoltage);
  }

  private void seedSensorPosition() {
    if (RobotBase.isSimulation()) {
      // The physics simualation needs to be run before this function attempts setting the sensor
      // position. This will happen in the next tick during periodic() otherwise.
      io.updateInputs(inputs);
    }

    var initialPosition = LifterPosition.UP;

    if (RobotBase.isSimulation()) {
      // TODO: Lifter visualization should start in the UP position
      initialPosition = LifterPosition.DOWN;
    }

    lastProfiledReference = initialPosition.state;

    // We want the down position to be a rotation of 0
    io.setEncoderPosition(new Rotation2d(initialPosition.state.position));
  }

  // TODO: Consider calling this from a command execute function
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

    // Send the new calculated voltage to the motors. voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    nextVoltage =
        loop.getU(0)
            + GRAVITY_FEEDFORWARD.calculate(
                lastProfiledReference.position, lastProfiledReference.velocity);
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
