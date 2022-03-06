// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.paths.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.wheel.Wheel;
import java.util.function.Supplier;

/**
 * An edit of {@link MecanumControllerCommand} that allows for dynamically providing {@link
 * Trajectory}s rather than just providing a single trajectory once during instantiation.
 *
 * <p>Additional edits have been made to make integrating with our {@link DriveSubsystem drive
 * subsystem} easier and less verbose.
 */
public class DynamicTrajectoryFollowCommand extends CommandBase {
  private final Timer timer = new Timer();
  private Trajectory trajectory;
  private final Supplier<Pose2d> pose;
  private final Supplier<Rotation2d> desiredRotation;
  private final Supplier<Trajectory> trajectorySupplier;
  private final DriveSubsystem driveSubsystem;

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
   * @param desiredRotation The angle that the robot should be facing. This is sampled at each time
   *     step.
   */
  public DynamicTrajectoryFollowCommand(
      Supplier<Trajectory> trajectorySupplier,
      Supplier<Pose2d> pose,
      Supplier<Rotation2d> desiredRotation,
      DriveSubsystem driveSubsystem) {
    trajectory = null;
    this.trajectorySupplier = trajectorySupplier;
    this.pose = pose;
    this.driveSubsystem = driveSubsystem;

    this.desiredRotation = desiredRotation;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    trajectory = trajectorySupplier.get();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    final double curTime = timer.get();

    final var desiredState = trajectory.sample(curTime);

    driveSubsystem.logTrajectoryPose(desiredState);

    final var targetChassisSpeeds =
        driveSubsystem.driveController.calculate(pose.get(), desiredState, desiredRotation.get());
    final var targetWheelSpeeds = driveSubsystem.kinematics.toWheelSpeeds(targetChassisSpeeds);

    targetWheelSpeeds.desaturate(Wheel.MAX_WHEEL_VELOCITY);

    driveSubsystem.setWheelSpeeds(targetWheelSpeeds);
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
