// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controller.DriveController;
import frc.robot.drive.commands.TeleopDriveCommand;
import frc.robot.imu.ImuSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * A high-level interface for the drivetrain.
 *
 * <p>Allows you to control all the wheels as a group (via {@link Drivebase}) as well as kinematics,
 * odometry, and trajectory helpers.
 */
public class DriveSubsystem extends SubsystemBase {
  private static final class Constants {
    // TODO: These values need to be properly recorded
    private static final TrapezoidProfile.Constraints MAX_ROTATION =
        new TrapezoidProfile.Constraints(3.13635666583381, Math.pow(3.13635666583381, 2));

    /** The acceptable amount of error between the robot's current pose and the desired pose. */
    private static final Pose2d POSE_TOLERANCE = new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(8));
  }

  public final MecanumDriveKinematics kinematics;

  public final TrajectoryConfig trajectoryConfig;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          1, 0, 0, Constants.MAX_ROTATION, frc.robot.Constants.PERIOD_SECONDS);

  // Used for following trajectories
  public final HolonomicDriveController driveController =
      new HolonomicDriveController(
          // X controller
          new PIDController(1, 0, 0, frc.robot.Constants.PERIOD_SECONDS),
          // Y controller
          new PIDController(1, 0, 0, frc.robot.Constants.PERIOD_SECONDS),
          thetaController);

  private final Drivebase drivebase;
  private final ImuSubsystem imuSubsystem;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(
      DriveController controller,
      ImuSubsystem imuSubsystem,
      WheelIO frontLeftIO,
      WheelIO frontRightIO,
      WheelIO rearLeftIO,
      WheelIO rearRightIO) {
    drivebase = new Drivebase(frontLeftIO, frontRightIO, rearLeftIO, rearRightIO);
    this.imuSubsystem = imuSubsystem;

    kinematics =
        new MecanumDriveKinematics(
            drivebase.frontLeft.positionToCenterOfRobot,
            drivebase.frontRight.positionToCenterOfRobot,
            drivebase.rearLeft.positionToCenterOfRobot,
            drivebase.rearRight.positionToCenterOfRobot);
    trajectoryConfig =
        new TrajectoryConfig(Drivebase.MAX_VELOCITY, Drivebase.MAX_ACCELERATION)
            .setKinematics(kinematics);

    setDefaultCommand(new TeleopDriveCommand(this, controller));

    driveController.setTolerance(Constants.POSE_TOLERANCE);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    drivebase.periodic();
  }

  public void driveTeleop(double xPercentage, double yPercentage, double thetaPercentage) {
    drivebase.setCartesianPercentages(
        xPercentage, yPercentage, thetaPercentage, imuSubsystem.getRotation());
  }

  /** Stops all the motors. */
  public void stopMotors() {
    drivebase.stopMotors();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(drivebase.getWheelSpeeds());
  }

  public double getLinearVelocity() {
    ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    return Math.sqrt(
        Math.pow(chassisSpeeds.vxMetersPerSecond, 2)
            + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    final var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    setWheelSpeeds(wheelSpeeds);
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return drivebase.getWheelSpeeds();
  }

  public void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    drivebase.setWheelSpeeds(wheelSpeeds);
  }

  /**
   * Used for showing a ghost robot of the expected position while following a trajectory. For
   * comparing with the actual position.
   */
  public void logTrajectoryPose(Trajectory.State state) {
    Logger.getInstance()
        .recordOutput(
            "Drive/TrajectoryPose",
            new double[] {
              state.poseMeters.getX(),
              state.poseMeters.getY(),
              state.poseMeters.getRotation().getRadians()
            });
  }

  // TODO: Delete this method, or replace it with another method for setting robot pose at match
  // start via trajectory starting pose
  /** Resets sensors to prepare for following a trajectory using its initial state. */
  public void resetSensorsForTrajectory(Trajectory.State initialTrajectoryState) {
    drivebase.zeroEncoders();
  }

  /** Resets sensors to prepare for following a trajectory. */
  public void resetSensorsForTrajectory(PathPlannerTrajectory trajectory) {
    resetSensorsForTrajectory(trajectory.getInitialState());
  }
}
