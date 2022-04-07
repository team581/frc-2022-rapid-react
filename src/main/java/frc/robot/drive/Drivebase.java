// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.misc.exceptions.UnknownTargetRobotException;

/**
 * This class should only be used within {@link DriveSubsystem}.
 *
 * <p>Allows you to control all the wheels as a group.
 */
class Drivebase extends SubsystemBase {
  private final Wheel frontLeft;
  private final Wheel frontRight;
  private final Wheel rearLeft;
  private final Wheel rearRight;

  public Drivebase(
      WheelIO frontLeftIO, WheelIO frontRightIO, WheelIO rearLeftIO, WheelIO rearRightIO) {

    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
      case TEST_2020_BOT:
        frontLeft = new Wheel(Corner.FRONT_LEFT, frontLeftIO);
        frontRight = new Wheel(Corner.FRONT_RIGHT, frontRightIO);
        rearLeft = new Wheel(Corner.REAR_LEFT, rearLeftIO);
        rearRight = new Wheel(Corner.REAR_RIGHT, rearRightIO);
        break;
      default:
        throw new UnknownTargetRobotException();
    }
  }

  @Override
  public void periodic() {
    frontLeft.periodic();
    frontRight.periodic();
    rearLeft.periodic();
    rearRight.periodic();
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        frontLeft.getVelocity(),
        frontRight.getVelocity(),
        rearLeft.getVelocity(),
        rearRight.getVelocity());
  }

  public void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    wheelSpeeds.desaturate(Wheel.MAX_WHEEL_VELOCITY);

    frontLeft.setDesiredVelocity(wheelSpeeds.frontLeftMetersPerSecond);
    frontRight.setDesiredVelocity(wheelSpeeds.frontRightMetersPerSecond);
    rearLeft.setDesiredVelocity(wheelSpeeds.rearLeftMetersPerSecond);
    rearRight.setDesiredVelocity(wheelSpeeds.rearRightMetersPerSecond);

    frontLeft.doVelocityControlLoop();
    frontRight.doVelocityControlLoop();
    rearLeft.doVelocityControlLoop();
    rearRight.doVelocityControlLoop();
  }

  /** Stops all the motors. */
  public void stopMotors() {
    setWheelSpeeds(new MecanumDriveWheelSpeeds());
  }
}
