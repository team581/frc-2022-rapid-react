// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DynamicMecanumControllerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.drive.Wheel;
import java.util.function.Supplier;
import lib.pathplanner.PPMecanumControllerCommand;

public class TrajectoryCommandFactory {
  private final DriveSubsystem driveSubsystem;

  /** Creates a new TrajectoryCommandFactory. */
  public TrajectoryCommandFactory(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  /**
   * @param trajectorySupplier A supplier for the trajectory to follow. Called once when the command
   *     is initialized.
   * @param getCurrentPose A function that returns the current pose of the robot (usually the
   *     distance to vision target or odometry).
   * @param desiredRotation The angle that the robot should be facing. This is sampled at each time
   *     step. This is essential because we use a holonomic drivetrain, and WPILib's trajectory
   *     generation assumes you are using a differential drivetrain.
   * @return A command to follow the trajectory provided by the supplier.
   */
  public Command createDynamicCommand(
      Supplier<Trajectory> trajectorySupplier,
      Supplier<Pose2d> getCurrentPose,
      Supplier<Rotation2d> desiredRotation) {
    return new DynamicMecanumControllerCommand(
        trajectorySupplier,
        getCurrentPose,
        driveSubsystem.kinematics,

        // Position PID controllers
        driveSubsystem.xPositionPid,
        driveSubsystem.yPositionPid,
        driveSubsystem.thetaPositionPid,

        // Custom rotation supplier because we're using holonomic drive
        desiredRotation,

        // Velocity control
        Wheel.MAX_WHEEL_VELOCITY,
        driveSubsystem::setWheelSpeeds,

        // Required subsystems
        driveSubsystem);
  }

  /**
   * @param trajectory The trajectory to follow.
   * @param getCurrentPose A function that returns the current pose of the robot (usually the
   *     distance to vision target or odometry).
   * @return A command to follow the trajectory provided by the supplier.
   */
  public Command createPPCommand(
      PathPlannerTrajectory trajectory, Supplier<Pose2d> getCurrentPose) {
    return new PPMecanumControllerCommand(
        trajectory,
        getCurrentPose,
        driveSubsystem.kinematics,

        // Position PID controllers
        driveSubsystem.xPositionPid,
        driveSubsystem.yPositionPid,
        driveSubsystem.thetaPositionPid,

        // Velocity control
        Wheel.MAX_WHEEL_VELOCITY,
        driveSubsystem::setWheelSpeeds,

        // Required subsystems
        driveSubsystem);
  }
}
