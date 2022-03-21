// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
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

  /** Disable feedback control. Only used for debugging. */
  private static final boolean DISABLE_FEEDBACK = false;

  // In this example we weight position much more highly than velocity, but this can be tuned to
  // balance the two.
  /** Maximum acceptable position error (in radians). */
  private static final double MAX_POSITION_ERROR =
      DISABLE_FEEDBACK ? 99999 : Units.degreesToRadians(5);

  /** Maximum acceptable angular velocity error (in radians per second). */
  private static final double MAX_VELOCITY_ERROR =
      DISABLE_FEEDBACK ? 99999 : Units.degreesToRadians(20);

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

  private final ArmIO io;
  private final Inputs inputs = new Inputs();

  private TrapezoidProfile.State lastProfiledReference = STARTING_POSITION.state;
  private ArmPosition desiredPosition = STARTING_POSITION;
  private double nextVoltage = 0;

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;

    final LinearSystem<N2, N1, N1> armPlant = io.getPlant();

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

    Logger.getInstance().processInputs("Arm", inputs);

    Logger.getInstance().recordOutput("Arm/DesiredPosition", desiredPosition.toString());
    Logger.getInstance().recordOutput("Arm/DesiredPositionRadians", desiredPosition.state.position);
    Logger.getInstance().recordOutput("Arm/PositionConstants/Up", ArmPosition.UP.state.position);
    Logger.getInstance()
        .recordOutput("Arm/PositionConstants/Down", ArmPosition.DOWN.state.position);

    // Avoid messing up the Kalman filter's state by making it believe we're using its output
    // voltages when the robot is disabled
    if (DriverStation.isEnabled()) {
      doPositionControlLoop();
    }

    Logger.getInstance().recordOutput("Arm/AtReference", atPosition(desiredPosition));
    Logger.getInstance()
        .recordOutput("Arm/Reference/DesiredPositionRadians", lastProfiledReference.position);
    Logger.getInstance()
        .recordOutput(
            "Arm/Reference/DesiredVelocityRadiansPerSecond", lastProfiledReference.velocity);
    Logger.getInstance()
        .recordOutput(
            "Arm/Reference/Error/PositionRadians",
            lastProfiledReference.position - inputs.position.getRadians());
    Logger.getInstance()
        .recordOutput(
            "Arm/Reference/Error/VelocityRadiansPerSecond",
            lastProfiledReference.velocity - inputs.velocityRadiansPerSecond);
    Logger.getInstance()
        .recordOutput("Arm/Loop/Observer/StateEstimate/VelocityRadiansPerSecond", loop.getXHat(0));
    Logger.getInstance()
        .recordOutput(
            "Arm/Loop/Observer/StateEstimate/AccelerationRadiansPerSecondSquared", loop.getXHat(1));
  }

  private void doPositionControlLoop() {
    // Get the next step of the trapezoid profile
    lastProfiledReference =
        new TrapezoidProfile(CONSTRAINTS, desiredPosition.state, lastProfiledReference)
            .calculate(Constants.PERIOD_SECONDS);
    loop.setNextR(lastProfiledReference.position, lastProfiledReference.velocity);

    // Correct our Kalman filter's state vector estimate with encoder data
    loop.correct(VecBuilder.fill(inputs.position.getRadians()));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter
    loop.predict(Constants.PERIOD_SECONDS);

    // Send the new calculated voltage to the motors. voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    nextVoltage =
        // TODO: Not sure why inverting the voltage here is required. Seems like just the
        // LinearSystemLoop needs it, not the motor itself.
        -loop.getU(0)
            + GRAVITY_FEEDFORWARD.calculate(
                lastProfiledReference.position, lastProfiledReference.velocity);
    io.setVoltage(nextVoltage);
    Logger.getInstance().recordOutput("Arm/DesiredAppliedVolts", nextVoltage);
  }

  /** Check if the arm is at the provided position. */
  public boolean atPosition(ArmPosition position) {
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

  /** Set the desired position of the arm. */
  public void setDesiredPosition(ArmPosition position) {
    desiredPosition = position;
  }
}
