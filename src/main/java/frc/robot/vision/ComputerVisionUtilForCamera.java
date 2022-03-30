// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import lib.wpilib.ComputerVisionUtil;

public class ComputerVisionUtilForCamera {
  private final Camera camera;

  public ComputerVisionUtilForCamera(Camera camera) {
    this.camera = camera;
  }

  /**
   * Algorithm from https://docs.limelightvision.io/en/latest/cs_estimating_distance.html Estimates
   * range to a target using the target's elevation. This method can produce more stable results
   * than SolvePNP when well tuned, if the full 6d robot pose is not required. Note that this method
   * requires the camera to have 0 roll (not be skewed clockwise or CCW relative to the floor), and
   * for there to exist a height differential between goal and camera. The larger this differential,
   * the more accurate the distance estimate will be.
   *
   * <p>Units can be converted using the {@link edu.wpi.first.math.util.Units} class.
   *
   * @param targetHeightMeters The physical height of the target off the floor in meters. This
   *     should be the height of whatever is being targeted (i.e. if the targeting region is set to
   * @param targetPitch The pitch of the target in the camera's lens. Positive values up.
   * @return The estimated distance to the target in meters.
   */
  public double calculateDistanceToTarget(double targetHeightMeters, Rotation2d targetPitch) {
    return ComputerVisionUtil.calculateDistanceToTarget(
        camera.heightFromFloor,
        targetHeightMeters,
        camera.angleOfElevation.getRadians(),
        targetPitch.getRadians());
  }

  /**
   * Estimate the position of the robot in the field.
   *
   * @param targetHeightMeters The physical height of the target off the floor in meters. This
   *     should be the height of whatever is being targeted (i.e. if the targeting region is set to
   *     top, this should be the height of the top of the target).
   * @param targetPitch The pitch of the target in the camera's lens. Positive values up.
   * @param targetYaw The observed yaw of the target. Note that this *must* be CCW-positive, and
   *     Photon returns CW-positive.
   * @param gyroAngle The current robot gyro angle, likely from odometry.
   * @param fieldToTarget A Pose2d representing the target position in the field coordinate system.
   * @return The position of the robot in the field.
   */
  public Pose2d estimateFieldToRobot(
      double targetHeightMeters,
      Rotation2d targetPitch,
      Rotation2d targetYaw,
      Rotation2d gyroAngle,
      Pose2d fieldToTarget) {
    return ComputerVisionUtil.estimateFieldToRobot(
        camera.heightFromFloor,
        targetHeightMeters,
        camera.angleOfElevation.getRadians(),
        targetPitch.getRadians(),
        targetYaw,
        gyroAngle,
        fieldToTarget,
        camera.transformToCenterOfRobot);
  }

  /**
   * Estimates the pose of the robot in the field coordinate system, given the position of the
   * target relative to the camera, the target relative to the field, and the robot relative to the
   * camera.
   *
   * @param cameraToTarget The position of the target relative to the camera.
   * @param fieldToTarget The position of the target in the field.
   * @return The position of the robot in the field.
   */
  public Pose2d estimateFieldToRobot(Transform2d cameraToTarget, Pose2d fieldToTarget) {
    return ComputerVisionUtil.estimateFieldToRobot(
        cameraToTarget, fieldToTarget, camera.transformToCenterOfRobot);
  }

  /**
   * Estimates a {@link Transform2d} that maps the camera position to the target position, using the
   * robot's gyro. Note that the gyro angle provided *must* line up with the field coordinate system
   * -- that is, it should read zero degrees when pointed towards the opposing alliance station, and
   * increase as the robot rotates CCW.
   *
   * @param cameraToTargetTranslation A Translation2d that encodes the x/y position of the target
   *     relative to the camera.
   * @param fieldToTarget A Pose2d representing the target position in the field coordinate system.
   * @param gyroAngle The current robot gyro angle, likely from odometry.
   * @return A Transform2d that takes us from the camera to the target.
   */
  public Transform2d estimateCameraToTarget(
      Translation2d cameraToTargetTranslation, Pose2d fieldToTarget, Rotation2d gyroAngle) {
    return ComputerVisionUtil.estimateCameraToTarget(
        cameraToTargetTranslation, fieldToTarget, gyroAngle);
  }

  /**
   * Estimates the pose of the camera in the field coordinate system, given the position of the
   * target relative to the camera, and the target relative to the field. This *only* tracks the
   * position of the camera, not the position of the robot itself.
   *
   * @param cameraToTarget The position of the target relative to the camera.
   * @param fieldToTarget The position of the target in the field.
   * @return The position of the camera in the field.
   */
  public Pose2d estimateFieldToCamera(Transform2d cameraToTarget, Pose2d fieldToTarget) {
    return ComputerVisionUtil.estimateFieldToCamera(cameraToTarget, fieldToTarget);
  }
}
