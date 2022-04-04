// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_upper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.Localization;
import frc.robot.misc.util.LoggingUtil;
import frc.robot.misc.util.PolarTranslation2d;
import frc.robot.vision.Camera;
import frc.robot.vision.ComputerVisionUtilForCamera;
import frc.robot.vision.VisionSubsystemBase;

import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class UpperHubVisionSubsystem extends VisionSubsystemBase {
  public enum Pipelines {
    UPPER_HUB(0),
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

  protected final Supplier<Rotation2d> robotRotation;

  private final UpperHubVisionTarget upperHub = new UpperHubVisionTarget();

  /** Creates a new UpperHubVisionSubsystem. */
  public UpperHubVisionSubsystem(UpperHubVisionIO io, ImuSubsystem imu) {
    super("UpperVision", io, Pipelines.DRIVER_MODE.index);

    robotRotation = imu::getRotation;
  }

  @Override
  public void periodic() {
    super.periodic();

    if (inputs.hasTargets) {
      // Draw the vision target on the odometry view if it's visible
      final var fieldToTarget = getFieldToTarget(inputs.tx);

      Logger.getInstance()
          .recordOutput(loggerName + "/VisionTarget/Pose", LoggingUtil.poseToArray(fieldToTarget));
      Logger.getInstance()
          .recordOutput(
              loggerName + "/VisionTarget/Translation",
              LoggingUtil.translationToArray(fieldToTarget.getTranslation()));
    }
  }

  /** Gets the current robot pose with visino processor latency factored in. */
  public Optional<TimestampedPose2d> getPastRobotPose() {
    if (!inputs.hasTargets) {
      return Optional.empty();
    }

    final var fieldToTarget = getFieldToTarget(inputs.tx);
    final var cameraToTarget = getCameraToTarget(inputs.tx, inputs.ty, fieldToTarget);
    final var fieldToRobot = VISION_UTIL.estimateFieldToRobot(cameraToTarget, fieldToTarget);

    if (!Localization.poseIsValid(fieldToRobot)) {
      return Optional.empty();
    }

    return Optional.of(new TimestampedPose2d(fieldToRobot, inputs.captureTimestamp));
  }

  /** @see {@link ComputerVisionUtil#estimateCameraToTarget(Translation2d, Pose2d, Rotation2d)} */
  private Transform2d getCameraToTarget(Rotation2d x, Rotation2d y, Pose2d fieldToTarget) {
    final var r = VISION_UTIL.calculateDistanceToTarget(upperHub.heightFromFloor, y);
    final var theta = x;
    final var cameraToTargetTranslation =
        new PolarTranslation2d(r, theta)
            // The vision target is 3D (a ring), not a flat shape against a wall. This means we need
            // to factor in the radius of the ring in our distance calculations. Adding this extra
            // pose ensures we  measure the distance from the camera to the center of the hub, not
            // the camera to the outer vision ring.
            .plus(UpperHubVisionTarget.TRANSLATION_FROM_OUTER_RING_TO_CENTER)
            .getTranslation2d();

    return VISION_UTIL.estimateCameraToTarget(
        cameraToTargetTranslation, fieldToTarget, robotRotation.get());
  }

  private Pose2d getFieldToTarget(Rotation2d x) {
    return new Pose2d(UpperHubVisionTarget.COORDINATES, new Rotation2d());
  }
}
