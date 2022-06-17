// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.controller.DriveController;
import frc.robot.drive.commands.TeleopDriveCommand;
import frc.robot.imu.ImuSubsystem;
import frc.robot.misc.exceptions.UnknownTargetRobotException;
import frc.robot.misc.util.LoggingUtil;
import org.littletonrobotics.junction.Logger;

/**
 * A high-level interface for the drivetrain.
 *
 * <p>Allows you to control all the wheels as a group (via {@link Drivebase}) as well as kinematics,
 * odometry, and trajectory helpers.
 */
public class DriveSubsystem extends SubsystemBase {
  public static final MecanumDriveKinematics KINEMATICS =
      // The distances of wheels to the center of the robot is currently the same for all robots
      new MecanumDriveKinematics(
          new Translation2d(0.285, 0.285),
          new Translation2d(0.285, -0.285),
          new Translation2d(-0.285, 0.285),
          new Translation2d(-0.285, -0.285));

  /** The robot's maximum velocity in meters per second. */
  public static final double MAX_VELOCITY;
  /** The robot's maximum acceleration in meters per second squared. */
  public static final double MAX_ACCELERATION;
  /**
   * The robot's maximum angular velocity in radians per second. Recorded by sitting still and
   * spinning as fast as possible.
   */
  public static final double MAX_ANGULAR_VELOCITY;
  /** The robot's maximum angular acceleration in radians per second squared. */
  public static final double MAX_ANGULAR_ACCELERATION;

  /**
   * The feedforward values from a SysID angular drivetrain characterization. These are different
   * than the usual linear drivetrain characterization. Linear kA is determined using your mass,
   * angular kA is determined using your mass distribution around the center of rotation.
   */
  private static final SimpleMotorFeedforward ROBOT_ANGULAR_FEEDFORWARD;

  static {
    switch (Constants.getRobot()) {
      case TEST_2020_BOT:
        ROBOT_ANGULAR_FEEDFORWARD = new SimpleMotorFeedforward(0.12211, 0.18984, 0.010019);
        break;
      case COMP_BOT:
      case SIM_BOT:
        // TODO: Use SysID to calculate the angular drivetrain feedforward constants
        ROBOT_ANGULAR_FEEDFORWARD = new SimpleMotorFeedforward(0.12211, 0.18984, 0.010019);
        break;
      default:
        throw new UnknownTargetRobotException();
    }

    final var maxWheelSpeedsForward =
        new MecanumDriveWheelSpeeds(
            Wheel.MAX_WHEEL_VELOCITY,
            Wheel.MAX_WHEEL_VELOCITY,
            Wheel.MAX_WHEEL_VELOCITY,
            Wheel.MAX_WHEEL_VELOCITY);
    final var maxChassisSpeedsForward = KINEMATICS.toChassisSpeeds(maxWheelSpeedsForward);
    final var maxWheelSpeedsSpinning =
        new MecanumDriveWheelSpeeds(
            Wheel.MAX_WHEEL_VELOCITY,
            -Wheel.MAX_WHEEL_VELOCITY,
            Wheel.MAX_WHEEL_VELOCITY,
            -Wheel.MAX_WHEEL_VELOCITY);
    final var maxChassisSpeedsSpinning = KINEMATICS.toChassisSpeeds(maxWheelSpeedsSpinning);

    MAX_VELOCITY = Math.abs(maxChassisSpeedsForward.vxMetersPerSecond);
    MAX_ANGULAR_VELOCITY = Math.abs(maxChassisSpeedsSpinning.omegaRadiansPerSecond);

    MAX_ACCELERATION = Wheel.MAX_ACCELERATION;
    MAX_ANGULAR_ACCELERATION = Wheel.MAX_VOLTAGE / ROBOT_ANGULAR_FEEDFORWARD.ka;
  }

  private static final TrapezoidProfile.Constraints MAX_ROTATION =
      new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION);

  /** The acceptable amount of error between the robot's current pose and the desired pose. */
  private static final Pose2d POSE_TOLERANCE = new Pose2d(0.15, 0.15, Rotation2d.fromDegrees(2));

  public final TrajectoryConfig trajectoryConfig;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(1, 0, 0, MAX_ROTATION, Constants.PERIOD_SECONDS);

  // Used for following trajectories
  public final HolonomicDriveController driveController =
      new HolonomicDriveController(
          // X controller
          new PIDController(1, 0, 0, Constants.PERIOD_SECONDS),
          // Y controller
          new PIDController(1, 0, 0, Constants.PERIOD_SECONDS),
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

    trajectoryConfig =
        new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION).setKinematics(KINEMATICS);

    setDefaultCommand(new TeleopDriveCommand(this, controller));

    driveController.setTolerance(POSE_TOLERANCE);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    drivebase.periodic();
  }

  public void driveTeleop(
      double sidewaysPercentage,
      double forwardPercentage,
      double thetaPercentage,
      boolean fieldRelative) {
    // Convert from CW+ to CCW+
    thetaPercentage *= -1;

    if (fieldRelative) {
      driveTeleop(
          sidewaysPercentage, forwardPercentage, thetaPercentage, imuSubsystem.getRotation());
    } else {
      driveTeleop(sidewaysPercentage, forwardPercentage, thetaPercentage, new Rotation2d());
    }
  }

  private void driveTeleop(
      double sidewaysPercentage,
      double forwardPercentage,
      double thetaPercentage,
      Rotation2d robotHeading) {
    final var goalHeadingDifferential =
        TeleopDriveCommand.MAX_TELEOP_TURN_RATE
            .times(Constants.PERIOD_SECONDS)
            .times(-thetaPercentage);
    final var newGoalHeading =
        thetaController.getGoal().position + goalHeadingDifferential.getRadians();

    final var thetaControllerVelocity =
        thetaController.calculate(imuSubsystem.getRotation().getRadians(), newGoalHeading);
    final var chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardPercentage * MAX_VELOCITY,
            -sidewaysPercentage * MAX_VELOCITY,
            thetaControllerVelocity,
            robotHeading);

    // TODO: Stop logging this after debugging is finished
    Logger.getInstance()
        .recordOutput("Drive/ThetaControllerVelocityRadiansPerSecond", thetaControllerVelocity);
    Logger.getInstance()
        .recordOutput("Drive/GoalHeadingRadians", thetaController.getGoal().position);

    setChassisSpeeds(chassisSpeeds);
  }

  /** Stops all the motors. */
  public void stopMotors() {
    drivebase.stopMotors();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    final var wheelSpeeds = KINEMATICS.toWheelSpeeds(chassisSpeeds);

    drivebase.setWheelSpeeds(wheelSpeeds);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return KINEMATICS.toChassisSpeeds(drivebase.getWheelSpeeds());
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return drivebase.getWheelSpeeds();
  }

  /**
   * Used for showing a ghost robot of the expected position while following a trajectory. For
   * comparing with the actual position.
   */
  public void logTrajectoryPose(Trajectory.State state) {
    logTrajectoryPose(state.poseMeters);
  }
  /**
   * Used for showing a ghost robot of the expected position while following a trajectory. For
   * comparing with the actual position.
   */
  public void logTrajectoryPose(Pose2d pose) {
    Logger.getInstance().recordOutput("Drive/TrajectoryPose", LoggingUtil.poseToArray(pose));
  }
}
