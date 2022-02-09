// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

// THE PLAN:

// Teleop:
// Use percentage control with the motors.
// There is no motion profiling or PID controllers or anything.
// Slew ratelimiting can be used to get similar effects as motion profiling.

// Auto:
// 1. Somehow, someone will call driveWithSpeeds() with a goal ChassisSpeeds (x, y, and omega).
// 2. Use kinematics to convert the ChassisSpeeds -> MecanumDriveWheelSpeeds
// 3. Closed loop velocity control with feedforward for each wheel

// Limelight:
// 1. The Limelight tells us the x, y, and omega error for aligning with a vision target.
//    This is a Pose2d.
// 2. Because we know the maximum possible velocity for x, y, and omega on our robot we can use that
//    to construct a ChassisSpeeds object.
//    If trying to go maximum speed is too fast, you can try using 50% of the max velocities before.
// 3. This object is given to driveWithSpeeds() (see above).

public class DriveSubsystem extends SubsystemBase implements Loggable {

  private static final class Constants {
    // Max of 1 rotation per second and max acceleration of 0.5 rotations
    // per second squared
    private static final TrapezoidProfile.Constraints MAX_ROTATION =
        new TrapezoidProfile.Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180));

    /** The diameter of the mecanum wheels on the drivebase, in meters. */
    private static final double MECANUM_WHEEL_DIAMETER = Units.inchesToMeters(5.97);

    private static final Wheel.EncoderConstants ENCODER_CONSTANTS =
        new Wheel.EncoderConstants(22197, 21998);

    private static final double MOTOR_VOLTAGE = 12;

    // Wheel velocity PID constants
    private static final double WHEEL_VELOCITY_PID_P = 3;
    private static final double WHEEL_VELOCITY_PID_I = 0;
    private static final double WHEEL_VELOCITY_PID_D = 0;

    // Wheel velocity feedforward constants
    // TODO: Re-run sysid to update these values
    private static final double WHEEL_VELOCITY_FF_S = 0.63584;
    private static final double WHEEL_VELOCITY_FF_V = 2.2138;
    private static final double WHEEL_VELOCITY_FF_A = 0.18561;
  }

  // TODO: Tune these values - currently they are just copy-pasted from 2020 (which is probably not
  // well-tuned either)
  private final PIDController xPid = new PIDController(0.01, 0, 0.0);
  private final PIDController yPid = new PIDController(0.01, 0, 0.0);
  private final ProfiledPIDController thetaPid =
      new ProfiledPIDController(0.01, 0, 0.0, Constants.MAX_ROTATION);

  public final HolonomicDriveController driveController =
      new HolonomicDriveController(xPid, yPid, thetaPid);

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.WHEEL_VELOCITY_FF_S,
          Constants.WHEEL_VELOCITY_FF_V,
          Constants.WHEEL_VELOCITY_FF_A);

  // #region wheels
  private final Wheel frontLeft =
      new Wheel(
          "frontLeft",
          new Wheel.MotorConstants(10, Constants.MOTOR_VOLTAGE),
          Constants.ENCODER_CONSTANTS,
          new Wheel.WheelConstants(
              new Translation2d(0.285, 0.285), Constants.MECANUM_WHEEL_DIAMETER),
          feedforward,
          new PIDController(
              Constants.WHEEL_VELOCITY_PID_P,
              Constants.WHEEL_VELOCITY_PID_I,
              Constants.WHEEL_VELOCITY_PID_D));
  public final Wheel frontRight =
      new Wheel(
          "frontRight",
          new Wheel.MotorConstants(11, Constants.MOTOR_VOLTAGE),
          Constants.ENCODER_CONSTANTS,
          new Wheel.WheelConstants(
              new Translation2d(0.285, -0.285), Constants.MECANUM_WHEEL_DIAMETER),
          feedforward,
          new PIDController(
              Constants.WHEEL_VELOCITY_PID_P,
              Constants.WHEEL_VELOCITY_PID_I,
              Constants.WHEEL_VELOCITY_PID_D));
  private final Wheel rearLeft =
      new Wheel(
          "rearLeft",
          new Wheel.MotorConstants(12, Constants.MOTOR_VOLTAGE),
          Constants.ENCODER_CONSTANTS,
          new Wheel.WheelConstants(
              new Translation2d(-0.285, 0.285), Constants.MECANUM_WHEEL_DIAMETER),
          feedforward,
          new PIDController(
              Constants.WHEEL_VELOCITY_PID_P,
              Constants.WHEEL_VELOCITY_PID_I,
              Constants.WHEEL_VELOCITY_PID_D));
  private final Wheel rearRight =
      new Wheel(
          "rearRight",
          new Wheel.MotorConstants(13, Constants.MOTOR_VOLTAGE),
          Constants.ENCODER_CONSTANTS,
          new Wheel.WheelConstants(
              new Translation2d(-0.285, -0.28), Constants.MECANUM_WHEEL_DIAMETER),
          feedforward,
          new PIDController(
              Constants.WHEEL_VELOCITY_PID_P,
              Constants.WHEEL_VELOCITY_PID_I,
              Constants.WHEEL_VELOCITY_PID_D));
  // #endregion

  private final MecanumDrive drive =
      new MecanumDrive(frontLeft.motor, rearLeft.motor, frontRight.motor, rearRight.motor);

  private final MecanumDriveKinematics kinematics =
      new MecanumDriveKinematics(
          frontLeft.wheelConstants.position,
          frontRight.wheelConstants.position,
          rearLeft.wheelConstants.position,
          rearRight.wheelConstants.position);

  private final GyroSubsystem gyroSubsystem;
  private final MecanumDriveOdometry odometry;

  public final TrajectoryConfig trajectoryConfig =
      new TrajectoryConfig(1, 1).setKinematics(kinematics);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(GyroSubsystem gyroSubsystem) {
    this.gyroSubsystem = gyroSubsystem;

    frontRight.motor.setInverted(true);
    rearRight.motor.setInverted(true);

    resetEncoders();
    odometry = new MecanumDriveOdometry(kinematics, gyroSubsystem.gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(gyroSubsystem.gyro.getRotation2d(), getWheelSpeeds());
  }

  public void driveTeleop(double xPercentage, double yPercentage, double thetaPercentage) {
    drive.driveCartesian(-yPercentage, xPercentage, thetaPercentage);
  }

  /** Stops all the motors. */
  public void stopMotors() {
    drive.stopMotor();
  }

  public void driveWithSpeeds(ChassisSpeeds chassisSpeeds) {
    final var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    frontLeft.setDesiredVelocity(wheelSpeeds.frontLeftMetersPerSecond);
    frontRight.setDesiredVelocity(wheelSpeeds.frontRightMetersPerSecond);
    rearLeft.setDesiredVelocity(wheelSpeeds.rearLeftMetersPerSecond);
    rearRight.setDesiredVelocity(wheelSpeeds.rearRightMetersPerSecond);

    frontLeft.drive();
    frontRight.drive();
    rearLeft.drive();
    rearRight.drive();

    drive.feed();
  }

  private MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        frontLeft.getVelocity(),
        frontRight.getVelocity(),
        rearLeft.getVelocity(),
        rearRight.getVelocity());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry() {
    resetEncoders();
    odometry.resetPosition(getPose(), gyroSubsystem.gyro.getRotation2d());
  }

  public void resetEncoders() {
    frontLeft.resetEncoder();
    frontRight.resetEncoder();
    rearLeft.resetEncoder();
    rearRight.resetEncoder();
  }
}
