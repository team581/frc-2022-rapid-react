// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.groups.drive.TeleopDriveCommand;
import frc.robot.subsystems.drive.Drivebase;
import frc.robot.subsystems.drive.Gyro;
import frc.robot.util.ControllerUtil;
import io.github.oblarg.oblog.Loggable;

/**
 * A high-level interface for the drivetrain.
 *
 * <p>Allows you to control all the wheels as a group (via {@link Drivebase}) as well as kinematics,
 * odometry, and trajectory helpers.
 */
public class DriveSubsystem extends SubsystemBase implements Loggable {
  private static final class Constants {
    // Max of 1 rotation per second and max acceleration of 0.5 rotations
    // per second squared
    private static final TrapezoidProfile.Constraints MAX_ROTATION =
        new TrapezoidProfile.Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180));
  }

  // Components of the drive subsystem to reduce how huge this file is
  private final Drivebase drivebase = new Drivebase();
  private final Gyro gyro = new Gyro();

  // Used for following trajectories
  public final PIDController xPositionPid = new PIDController(1, 0, 0);
  public final PIDController yPositionPid = new PIDController(1, 0, 0);
  public final ProfiledPIDController thetaPositionPid =
      new ProfiledPIDController(1, 0, 0, Constants.MAX_ROTATION);

  public final MecanumDriveKinematics kinematics =
      new MecanumDriveKinematics(
          drivebase.frontLeft.positionToCenterOfRobot,
          drivebase.frontRight.positionToCenterOfRobot,
          drivebase.rearLeft.positionToCenterOfRobot,
          drivebase.rearRight.positionToCenterOfRobot);

  private final MecanumDriveOdometry odometry =
      new MecanumDriveOdometry(kinematics, gyro.sensor.getRotation2d());

  public final TrajectoryConfig trajectoryConfig =
      new TrajectoryConfig(Drivebase.Constants.MAX_VELOCITY, Drivebase.Constants.MAX_ACCELERATION)
          .setKinematics(kinematics);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(ControllerUtil controller) {
    setDefaultCommand(new TeleopDriveCommand(this, controller));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(gyro.sensor.getRotation2d(), drivebase.getWheelSpeeds());
  }

  public void driveTeleop(double xPercentage, double yPercentage, double thetaPercentage) {
    drivebase.setCartesianPercentages(xPercentage, yPercentage, thetaPercentage);
  }

  /** Stops all the motors. */
  public void stopMotors() {
    drivebase.stopMotors();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    final var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    setWheelSpeeds(wheelSpeeds);
  }

  public void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    drivebase.setWheelSpeeds(wheelSpeeds);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry() {
    drivebase.resetEncoders();
    odometry.resetPosition(getPose(), gyro.sensor.getRotation2d());
  }
}
