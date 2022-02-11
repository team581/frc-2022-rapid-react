// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * An edit of {@link MecanumControllerCommand} that allows for dynamically providing {@link
 * Trajectory}s rather than just providing a single trajectory once during instantiation.
 */
public class DynamicMecanumControllerCommand extends CommandBase {
  private final Timer timer = new Timer();
  private Trajectory trajectory;
  private final Supplier<Pose2d> pose;
  private final MecanumDriveKinematics kinematics;
  private final HolonomicDriveController controller;
  private final Supplier<Rotation2d> desiredRotation;
  private final double maxWheelVelocityMetersPerSecond;
  private final Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds;
  private final Supplier<Trajectory> trajectorySupplier;

  /**
   * Constructs a new DynamicMecanumControllerCommand that when executed will follow the provided
   * trajectory. The user should implement a velocity PID on the desired output wheel velocities.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with non-stationary end-states.
   *
   * @param trajectorySupplier A supplier that returns a trajectory to use for this scheduling of
   *     the command. Called once during initialization.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param desiredRotation The angle that the robot should be facing. This is sampled at each time
   *     step.
   * @param maxWheelVelocityMetersPerSecond The maximum velocity of a drivetrain wheel.
   * @param outputWheelSpeeds A MecanumDriveWheelSpeeds object containing the output wheel speeds.
   * @param requirements The subsystems to require.
   */
  public DynamicMecanumControllerCommand(
      Supplier<Trajectory> trajectorySupplier,
      Supplier<Pose2d> pose,
      MecanumDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredRotation,
      double maxWheelVelocityMetersPerSecond,
      Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds,
      Subsystem... requirements) {
    trajectory = null;
    this.trajectorySupplier = trajectorySupplier;
    this.pose = pose;
    this.kinematics = kinematics;

    controller = new HolonomicDriveController(xController, yController, thetaController);

    this.desiredRotation = desiredRotation;

    this.maxWheelVelocityMetersPerSecond = maxWheelVelocityMetersPerSecond;

    this.outputWheelSpeeds = outputWheelSpeeds;

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    trajectory = trajectorySupplier.get();
    timer.reset();
    timer.start();
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    final double curTime = timer.get();

    final var desiredState = trajectory.sample(curTime);

    final var targetChassisSpeeds =
        controller.calculate(pose.get(), desiredState, desiredRotation.get());
    final var targetWheelSpeeds = kinematics.toWheelSpeeds(targetChassisSpeeds);

    targetWheelSpeeds.desaturate(maxWheelVelocityMetersPerSecond);

    outputWheelSpeeds.accept(targetWheelSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
