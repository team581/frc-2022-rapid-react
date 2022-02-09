// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.LimelightTrajectoryGenerator;
import frc.robot.vision.targets.LimelightVisionTarget;
import java.util.function.Supplier;

/** Create commands to align with a vision target using the Limelight. */
public class AlignWithLimelightCommandFactory {
  private final LimelightTrajectoryGenerator trajectoryGenerator;
  private final TrajectoryCommandFactory commandFactory;

  /** Creates a new AlignWithLimelightCommandFactory. */
  public AlignWithLimelightCommandFactory(DriveSubsystem driveSubsystem) {
    trajectoryGenerator = new LimelightTrajectoryGenerator(driveSubsystem);
    commandFactory = new TrajectoryCommandFactory(driveSubsystem);
  }

  /**
   * Generates a {@link Command command} to align with a {@link LimelightVisionTarget Limelight
   * vision target} by following a generated trajectory.
   *
   * @param visionTarget The vision target to use for determining the robot's position
   * @param goal The goal position relative to the vision target
   * @param rotationSupplier The angle that the robot should be facing. This is sampled at each time
   *     step. An angle of 0 is facing directly toward the vision target. This is because the vision
   *     target has a pose of Pose2d(0, 0, 0).
   * @return A command that will move the robot to the goal position
   */
  public Command generateCommand(
      LimelightVisionTarget visionTarget, Pose2d goal, Supplier<Rotation2d> rotationSupplier) {
    final var trajectory = trajectoryGenerator.generateTrajectory(visionTarget, goal);

    return commandFactory.createCommand(trajectory, visionTarget::getRobotPose, rotationSupplier);
  }
}
