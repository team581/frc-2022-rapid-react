// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.subsystems.DriveSubsystem;
import io.github.oblarg.oblog.Loggable;

/**
 * This class should only be used within {@link DriveSubsystem}.
 *
 * <p>Allows you to control all the wheels as a group.
 */
public class Drivebase implements Loggable {
  public static final class Constants {
    /** The robot's maximum velocity in meters per second. */
    public static final double MAX_VELOCITY = 4.5;
    /** The robot's maximum acceleration in meters per second squared. */
    public static final double MAX_ACCELERATION = 16;
  }

  public final Wheel frontLeft = new Wheel("frontLeft", 10, new Translation2d(0.285, 0.285));
  public final Wheel frontRight = new Wheel("frontRight", 11, new Translation2d(0.285, -0.285));
  public final Wheel rearLeft = new Wheel("rearLeft", 12, new Translation2d(-0.285, 0.285));
  public final Wheel rearRight = new Wheel("rearRight", 13, new Translation2d(-0.285, -0.28));

  public final MecanumDrive drive =
      new MecanumDrive(frontLeft.motor, rearLeft.motor, frontRight.motor, rearRight.motor);

  public Drivebase() {
    frontRight.motor.setInverted(true);
    rearRight.motor.setInverted(true);

    resetEncoders();
  }

  public void setCartesianPercentages(
      double xPercentage, double yPercentage, double thetaPercentage) {
    drive.driveCartesian(-yPercentage, xPercentage, thetaPercentage);
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

  public void resetEncoders() {
    frontLeft.resetEncoder();
    frontRight.resetEncoder();
    rearLeft.resetEncoder();
    rearRight.resetEncoder();
  }
}
