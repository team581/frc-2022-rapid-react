// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drive.DriveSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.misc.util.LoggingUtil;
import frc.robot.vision_cargo.CargoVisionSubsystem;
import frc.robot.vision_cargo.UpperHubVisionTarget;
import frc.robot.vision_upper.TimestampedPose2d;
import java.util.Optional;
import lib.wpilib.MecanumDrivePoseEstimator;
import org.ejml.data.SingularMatrixException;
import org.littletonrobotics.junction.Logger;

/** Used for combining sensor data to localize the robot on the field. */
public class Localization extends SubsystemBase {
  /** Returns whether a translation is within the bounds of the field. */
  public static boolean translationIsValid(Translation2d translation) {
    final var x = translation.getX();
    final var y = translation.getY();

    return x >= 0 && y >= 0 && x <= Constants.FIELD_WIDTH && y <= Constants.FIELD_HEIGHT;
  }

  /** Returns whether a pose is within the bounds of the field. */
  public static boolean poseIsValid(Pose2d pose) {
    return translationIsValid(pose.getTranslation());
  }

  private final DriveSubsystem driveSubsystem;
  private final CargoVisionSubsystem visionSubsystem;
  private final ImuSubsystem imuSubsystem;
  private final MecanumDrivePoseEstimator poseEstimator;
  private final MecanumDriveOdometry odometry;

  int odometryErrorCount = 0;
  int visionErrorCount = 0;

  public Localization(
      DriveSubsystem driveSubsystem,
      CargoVisionSubsystem visionSubsystem,
      ImuSubsystem imuSubsystem,
      Pose2d initialRobotPose) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.imuSubsystem = imuSubsystem;

    final var initialHeading = imuSubsystem.getRotation();

    poseEstimator =
        new MecanumDrivePoseEstimator(
            // Initial heading
            initialHeading,
            // Initial position
            initialRobotPose,
            DriveSubsystem.KINEMATICS,
            // Standard deviations of wheel odometry x, y, and theta
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            // Standard deviation of gyroscope heading
            VecBuilder.fill(Units.degreesToRadians(0.01)),
            // Vision measurement standard deviations
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)),
            Constants.PERIOD_SECONDS);

    odometry =
        new MecanumDriveOdometry(DriveSubsystem.KINEMATICS, initialHeading, initialRobotPose);
  }

  public Localization(
      DriveSubsystem driveSubsystem,
      CargoVisionSubsystem visionSubsystem,
      ImuSubsystem imuSubsystem) {
    this(
        driveSubsystem,
        visionSubsystem,
        imuSubsystem,
        // Default to being in the center of the field
        // This should only really be used for debugging
        new Pose2d(UpperHubVisionTarget.COORDINATES, new Rotation2d()));
  }

  @Override
  public void periodic() {
    final var rotation = imuSubsystem.getRotation();
    final var wheelSpeeds = driveSubsystem.getWheelSpeeds();

    final var odometryPose = odometry.update(rotation, wheelSpeeds);

    // The robot's position, just using odometry
    Logger.getInstance()
        .recordOutput("Localization/OdometryPose", LoggingUtil.poseToArray(odometryPose));

    try {
      final var localizationPose = poseEstimator.update(imuSubsystem.getRotation(), wheelSpeeds);

      // The robot's position using both odometry and vision
      Logger.getInstance()
          .recordOutput("Localization/RobotPose", LoggingUtil.poseToArray(localizationPose));
    } catch (SingularMatrixException e) {
      // Ignore errors that occur from an unsolvable matrix due to uncountable numbers
      // This seems to periodically happen when driving around IRL with the camera on
      // This is probably from the Cholesky decomposition not finding a solution

      odometryErrorCount++;
    }

    try {
      final var optionalVisionPose = getRawVisionPose();
      if (optionalVisionPose.isPresent()) {
        final var visionPose = optionalVisionPose.get();

        // The robot's position, just using the vision data
        Logger.getInstance()
            .recordOutput("Localization/VisionPose", LoggingUtil.poseToArray(visionPose.pose));

        poseEstimator.addVisionMeasurement(visionPose.pose, visionPose.timestamp);
      }
    } catch (SingularMatrixException e) {
      // Ignore errors that occur from an unsolvable matrix due to uncountable numbers
      // This seems to periodically happen when driving around IRL with the camera on
      // This is probably from the Cholesky decomposition not finding a solution

      visionErrorCount++;
    }

    Logger.getInstance().recordOutput("Localization/Errors/OdometryErrorCount", odometryErrorCount);
    Logger.getInstance().recordOutput("Localization/Errors/VisionErrorCount", visionErrorCount);
  }

  /** Returns the robot's position using both vision and odometry. */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d robotPose) {
    imuSubsystem.resetHeadingTo(robotPose.getRotation());
    poseEstimator.resetPosition(robotPose, imuSubsystem.getRotation());
    odometry.resetPosition(robotPose, imuSubsystem.getRotation());
  }

  /**
   * Returns the robot pose from the vision system directly, if available. You probably want to use
   * {@link #getPose()} instead.
   */
  private Optional<TimestampedPose2d> getRawVisionPose() {
    return visionSubsystem.getPastRobotPose();
  }
}
