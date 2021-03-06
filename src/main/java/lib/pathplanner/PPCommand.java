// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.pathplanner;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.DriveSubsystem;
import frc.robot.localization.Localization;

/**
 * A command that uses a {@link HolonomicDriveController} to follow a trajectory {@link
 * PathPlannerTrajectory} with our {@link DriveSubsystem drive subsystem}.
 */
public class PPCommand extends CommandBase {
  private final Timer timer = new Timer();
  private final PathPlannerTrajectory trajectory;
  private final DriveSubsystem driveSubsystem;
  private final Localization localization;

  /**
   * Constructs a new PPMecanumControllerCommand that when executed will follow the provided
   * trajectory. Each {@link Wheel wheel} will have its desired velocity set according to the
   * current state of the trajectory.
   *
   * <p>Note: The controllers will *not* set the desired velocity to zero upon completion of the
   * path - this is left to the user, since it is not appropriate for paths with non-stationary
   * end-states.
   *
   * @param trajectory The Pathplanner trajectory to follow.
   * @param driveSubsystem The drive subsystem to use.
   * @param localization The source of localization data.
   */
  public PPCommand(
      PathPlannerTrajectory trajectory, DriveSubsystem driveSubsystem, Localization localization) {
    this.trajectory = trajectory;
    this.driveSubsystem = driveSubsystem;
    this.localization = localization;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double curTime = timer.get();
    var desiredState = (PathPlannerState) trajectory.sample(curTime);

    driveSubsystem.logTrajectoryPose(desiredState);

    var targetChassisSpeeds =
        driveSubsystem.driveController.calculate(
            localization.getPose(), desiredState, desiredState.holonomicRotation);

    driveSubsystem.setChassisSpeeds(targetChassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    driveSubsystem.stopMotors();
  }

  @Override
  public boolean isFinished() {
    // TODO: Allow ending the command early if the trajectory is finished before the timer
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
