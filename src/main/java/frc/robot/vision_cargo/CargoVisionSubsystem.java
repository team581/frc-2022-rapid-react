// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_cargo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.Localization;
import frc.robot.misc.util.PolarTranslation2d;
import frc.robot.vision.Camera;
import frc.robot.vision.ComputerVisionUtilForCamera;
import frc.robot.vision.VisionSubsystemBase;
import frc.robot.vision_cargo.CargoVisionTarget.Color;
import frc.robot.vision_upper.TimestampedPose2d;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class CargoVisionSubsystem extends VisionSubsystemBase {
  public enum Pipelines {
    RED_CARGO(0),
    BLUE_CARGO(1),
    UPPER_HUB(2),
    LOADING_BAY(3),
    DRIVER_MODE(9);

    public final int index;

    Pipelines(final int index) {
      this.index = index;
    }
  }

  private static final Camera CAMERA;
  private static final ComputerVisionUtilForCamera VISION_UTIL;

  static {
    switch (Constants.getRobot()) {
      case TEST_2020_BOT:
      case SIM_BOT:
        CAMERA =
            new Camera(
                Units.inchesToMeters(15.5),
                new Rotation2d(0),
                new Transform2d(new Translation2d(0, 0.285), new Rotation2d(0)));
        break;
      default:
        CAMERA =
            new Camera(
                Units.feetToMeters(1),
                new Rotation2d(0),
                new Transform2d(new Translation2d(0, 0), new Rotation2d(0)));
        break;
    }

    VISION_UTIL = new ComputerVisionUtilForCamera(CAMERA);
  }

  public final LoadingBayVisionTarget loadingBay = new LoadingBayVisionTarget(this);
  private final UpperHubVisionTarget upperHub = new UpperHubVisionTarget(this);
  private final CargoVisionTarget redCargo =
      new CargoVisionTarget(this, CargoVisionTarget.Color.RED);
  private final CargoVisionTarget blueCargo =
      new CargoVisionTarget(this, CargoVisionTarget.Color.BLUE);
  private final ImuSubsystem imu;
  private CargoVisionTarget ourCargoVisionTarget;
  private CargoVisionTarget opponentCargoVisionTarget;

  /** Creates a new CargoVisionSubsystem. */
  public CargoVisionSubsystem(CargoVisionIO io, ImuSubsystem imu) {
    super("CargoVision", io, Pipelines.DRIVER_MODE.index);

    this.imu = imu;
  }

  @Override
  public void periodic() {
    super.periodic();

    final var optionalTranslation = upperHub.getTranslationFromCamera();
    if (optionalTranslation.isPresent()) {
      final var translation = optionalTranslation.get();
      Logger.getInstance().recordOutput(loggerName + "/DistanceToHubMeters", translation.getR());
      Logger.getInstance()
          .recordOutput(loggerName + "/AngleToHubRadians", translation.getTheta().getRadians());

      final var optionalVisionPose = getRobotPose();
      if (optionalVisionPose.isPresent()) {
        Logger.getInstance()
            .recordOutput(
                loggerName + "/VisionTargetPose",
                new double[] {UpperHubVisionTarget.POSE.getX(), UpperHubVisionTarget.POSE.getY()});
      }
    }
  }

  @Override
  public ComputerVisionUtilForCamera getVisionUtil() {
    return VISION_UTIL;
  }

  public CargoVisionTarget getOurCargoVisionTarget() {
    return ourCargoVisionTarget;
  }

  public CargoVisionTarget getOpponentCargoVisionTarget() {
    return opponentCargoVisionTarget;
  }

  /** Sets the {@link CargoVisionTarget}s in this class based on what our team's alliance is. */
  public void setAlliances(Alliance ourAlliance) {
    if (ourAlliance == Alliance.Invalid) {
      return;
    }

    final var ourAllianceColor = ourAlliance == Alliance.Blue ? Color.BLUE : Color.RED;
    final var opponentAllianceColor = ourAlliance == Alliance.Blue ? Color.RED : Color.BLUE;

    ourCargoVisionTarget = getTargetForAlliance(ourAllianceColor);
    opponentCargoVisionTarget = getTargetForAlliance(opponentAllianceColor);
  }

  /**
   * Get the pose of the robot using the upper hub vision target and the gyroscope, if available.
   */
  public Optional<TimestampedPose2d> getRobotPose() {
    final var optionalCameraToHub = upperHub.getTranslationFromCamera();
    if (optionalCameraToHub.isEmpty()) {
      return Optional.empty();
    }

    // Get the angle and distance as reported by the Limelight from the camera to the hub
    final var cameraToHubRelative = optionalCameraToHub.get();
    final Rotation2d targetYaw = cameraToHubRelative.getTheta();
    final var targetPitch = getY();
    final var gyroAngle = imu.getRotation();
    final var upperHubTransform = UpperHubVisionTarget.POSE;

    // Estimate the robot's field position using the Hub height, pitch & yaw to the Hub, robot facing, and position of the Hub
    // Note: This also takes into account the transform from the camera to the robot, and the camera height.
    // As far as I know, the angle will be ignored in all the estimateFieldToRobot math because
    // all that matters is where it is, not where it's facing.
    final var fieldPose = new Pose2d(upperHubTransform, new Rotation2d());

    final var robotPose = getVisionUtil().estimateFieldToRobot(upperHub.heightFromFloor, targetPitch, targetYaw, gyroAngle, fieldPose);

    // Only return this pose if it's valid (i.e. Inside the field)
    if (Localization.poseIsValid(robotPose)) {
      return Optional.of(new TimestampedPose2d(robotPose, inputs.captureTimestamp));
    }

    // Invalid pose, you can't be outside of the field
    return Optional.empty();
  }

  /** Gets the {@link CargoVisionTarget} instance for the provided alliance (red or blue). */
  private CargoVisionTarget getTargetForAlliance(CargoVisionTarget.Color alliance) {
    if (alliance == CargoVisionTarget.Color.RED) {
      return redCargo;
    }

    return blueCargo;
  }
}
