// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants;
import frc.robot.drive.wheel.Wheel;
import frc.robot.drive.wheel.WheelIO;

/**
 * This class should only be used within {@link DriveSubsystem}.
 *
 * <p>Allows you to control all the wheels as a group.
 */
public class Drivebase {
  /** The robot's maximum velocity in meters per second. */
  public static final double MAX_VELOCITY;
  /** The robot's maximum acceleration in meters per second squared. */
  public static final double MAX_ACCELERATION;

  static {
    switch (Constants.getRobot()) {
      case TEST_2020_BOT:
        MAX_VELOCITY = 4.5;
        MAX_ACCELERATION = 16;
        break;
      case COMP_BOT:
      case SIM_BOT:
        // TODO: These need to be measured
        MAX_VELOCITY = 4.5;
        MAX_ACCELERATION = 16;
        break;
      default:
        throw new IllegalStateException("Unknown target robot");
    }
  }

  public final Wheel frontLeft;
  public final Wheel frontRight;
  public final Wheel rearLeft;
  public final Wheel rearRight;

  public final MecanumDrive drive;

  public Drivebase(
      WheelIO frontLeftIO, WheelIO frontRightIO, WheelIO rearLeftIO, WheelIO rearRightIO) {

    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        // TODO: These need to be measured
        frontLeft = new Wheel("FrontLeft", frontLeftIO, new Translation2d(0.285, 0.285));
        frontRight = new Wheel("FrontRight", frontRightIO, new Translation2d(0.285, -0.285));
        rearLeft = new Wheel("RearLeft", rearLeftIO, new Translation2d(-0.285, 0.285));
        rearRight = new Wheel("RearRight", rearRightIO, new Translation2d(-0.285, -0.285));
        break;
      case TEST_2020_BOT:
        frontLeft = new Wheel("FrontLeft", frontLeftIO, new Translation2d(0.285, 0.285));
        frontRight = new Wheel("FrontRight", frontRightIO, new Translation2d(0.285, -0.285));
        rearLeft = new Wheel("RearLeft", rearLeftIO, new Translation2d(-0.285, 0.285));
        rearRight = new Wheel("RearRight", rearRightIO, new Translation2d(-0.285, -0.285));
        break;
      default:
        throw new IllegalStateException("Unknown target robot");
    }

    drive =
        new MecanumDrive(
            frontLeftIO.getMotorController(),
            rearLeftIO.getMotorController(),
            frontRightIO.getMotorController(),
            rearRightIO.getMotorController());

    zeroEncoders();
  }

  public void periodic() {
    frontLeft.periodic();
    frontRight.periodic();
    rearLeft.periodic();
    rearRight.periodic();
  }

  public void setCartesianPercentages(
      double xPercentage, double yPercentage, double thetaPercentage) {
    drive.driveCartesian(yPercentage, xPercentage, thetaPercentage);
  }

  public void setCartesianPercentages(
      double xPercentage, double yPercentage, double thetaPercentage, Rotation2d currentRotation) {
    drive.driveCartesian(yPercentage, xPercentage, thetaPercentage, currentRotation.getDegrees());
  }

  public void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    frontLeft.setDesiredVelocity(wheelSpeeds.frontLeftMetersPerSecond);
    frontRight.setDesiredVelocity(wheelSpeeds.frontRightMetersPerSecond);
    rearLeft.setDesiredVelocity(wheelSpeeds.rearLeftMetersPerSecond);
    rearRight.setDesiredVelocity(wheelSpeeds.rearRightMetersPerSecond);

    frontLeft.doVelocityControlLoop();
    frontRight.doVelocityControlLoop();
    rearLeft.doVelocityControlLoop();
    rearRight.doVelocityControlLoop();

    drive.feed();
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        frontLeft.getVelocity(),
        frontRight.getVelocity(),
        rearLeft.getVelocity(),
        rearRight.getVelocity());
  }

  /** Stops all the motors. */
  public void stopMotors() {
    setWheelSpeeds(new MecanumDriveWheelSpeeds());
    drive.stopMotor();
  }

  public void zeroEncoders() {
    frontLeft.zeroEncoder();
    frontRight.zeroEncoder();
    rearLeft.zeroEncoder();
    rearRight.zeroEncoder();
  }
}
