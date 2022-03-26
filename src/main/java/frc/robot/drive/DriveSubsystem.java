// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controller.DriveController;
import frc.robot.drive.commands.TeleopDriveCommand;
import frc.robot.imu.ImuSubsystem;
import frc.robot.vision_upper.UpperHubVisionSubsystem;
import lib.wpilib.MecanumDrivePoseEstimator;
import org.littletonrobotics.junction.Logger;

/**
 * A high-level interface for the drivetrain.
 *
 * <p>Allows you to control all the wheels as a group (via {@link Drivebase}) as well as kinematics,
 * odometry, and trajectory helpers.
 */
public class DriveSubsystem extends SubsystemBase {
  private static final class Constants {
    // Max of 1 rotation per second and max acceleration of 0.5 rotations
    // per second squared
    private static final TrapezoidProfile.Constraints MAX_ROTATION =
        new TrapezoidProfile.Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180));

    /** The acceptable amount of error between the robot's current pose and the desired pose. */
    private static final Pose2d POSE_TOLERANCE = new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(5));
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

  private final ImuSubsystem imu;

  private final MecanumDrivePoseEstimator poseEstimator;

  private final UpperHubVisionSubsystem vision;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(
      DriveController controller,
      ImuSubsystem imu,
      UpperHubVisionSubsystem vision,
      WheelIO frontLeftIO,
      WheelIO frontRightIO,
      WheelIO rearLeftIO,
      WheelIO rearRightIO) {
    this.imu = imu;
    this.vision = vision;
    drivebase = new Drivebase(frontLeftIO, frontRightIO, rearLeftIO, rearRightIO);

    kinematics =
        new MecanumDriveKinematics(
            drivebase.frontLeft.positionToCenterOfRobot,
            drivebase.frontRight.positionToCenterOfRobot,
            drivebase.rearLeft.positionToCenterOfRobot,
            drivebase.rearRight.positionToCenterOfRobot);
    trajectoryConfig =
        new TrajectoryConfig(Drivebase.MAX_VELOCITY, Drivebase.MAX_ACCELERATION)
            .setKinematics(kinematics);
    poseEstimator =
        new MecanumDrivePoseEstimator(
            // Initial heading
            imu.getRotation(),
            // Initial position
            // TODO: Allow this to be configured based on autonomous routine starting location
            new Pose2d(),
            kinematics,
            // Standard deviations of wheel odometry x, y, and theta
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            // Standard deviation of gyroscope heading
            VecBuilder.fill(Units.degreesToRadians(0.01)),
            // Vision measurement standard deviations
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)),
            frc.robot.Constants.PERIOD_SECONDS);

    setDefaultCommand(new TeleopDriveCommand(this, controller));

    driveController.setTolerance(Constants.POSE_TOLERANCE);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    drivebase.periodic();

    poseEstimator.update(imu.getRotation(), drivebase.getWheelSpeeds());

    final var optionalVisionPose = vision.getRobotPose();
    if (optionalVisionPose.isPresent()) {
      final var visionPose = optionalVisionPose.get();
      poseEstimator.addVisionMeasurement(visionPose.pose, visionPose.timestamp);
    }

    final var pose = getPose();
    // The robot's position
    Logger.getInstance()
        .recordOutput(
            "Drive/RobotPose",
            new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()});
  }

  public void driveTeleop(double xPercentage, double yPercentage, double thetaPercentage) {
    drivebase.setCartesianPercentages(xPercentage, yPercentage, thetaPercentage, imu.getRotation());
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

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
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
