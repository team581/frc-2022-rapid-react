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
import frc.robot.misc.util.LoggingUtil;
import frc.robot.misc.util.PolarTranslation2d;
import frc.robot.vision.Camera;
import frc.robot.vision.ComputerVisionUtilForCamera;
import frc.robot.vision.VisionSubsystemBase;
import frc.robot.vision_cargo.CargoVisionTarget.Color;
import frc.robot.vision_upper.TimestampedPose2d;
import java.util.Optional;
import java.util.function.Supplier;
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

  private final UpperHubVisionTarget upperHub = new UpperHubVisionTarget();
  private final CargoVisionTarget redCargo =
      new CargoVisionTarget(this, CargoVisionTarget.Color.RED);
  private final CargoVisionTarget blueCargo =
      new CargoVisionTarget(this, CargoVisionTarget.Color.BLUE);

  protected final Supplier<Rotation2d> robotRotation;

  private CargoVisionTarget ourCargoVisionTarget;
  private CargoVisionTarget opponentCargoVisionTarget;

  /** Creates a new CargoVisionSubsystem. */
  public CargoVisionSubsystem(CargoVisionIO io, ImuSubsystem imu) {
    super("CargoVision", io, Pipelines.DRIVER_MODE.index);

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

  public CargoVisionTarget getOurCargoVisionTarget() {
    return ourCargoVisionTarget;
  }

  public CargoVisionTarget getOpponentCargoVisionTarget() {
    return opponentCargoVisionTarget;
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
    final var theta = x.unaryMinus();
    final var cameraToTargetTranslation =
        new PolarTranslation2d(r, theta)
            // The vision target is 3D (a ring), not a flat shape against a wall. This means we need
            // to factor in the radius of the ring in our distance calculations. Adding this extra
            // pose ensures we  measure the distance from the camera to the center of the hub, not
            // the camera to the outer vision ring.
            .plus(UpperHubVisionTarget.TRANSLATION_FROM_OUTER_RING_TO_CENTER)
            .getTranslation2d()
            .unaryMinus();

    return VISION_UTIL.estimateCameraToTarget(
        cameraToTargetTranslation, fieldToTarget, robotRotation.get());
  }

  private Pose2d getFieldToTarget(Rotation2d x) {
    return new Pose2d(
        UpperHubVisionTarget.COORDINATES,
        robotRotation.get().plus(Rotation2d.fromDegrees(180)).minus(x));
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

  /** Gets the {@link CargoVisionTarget} instance for the provided alliance (red or blue). */
  private CargoVisionTarget getTargetForAlliance(CargoVisionTarget.Color alliance) {
    if (alliance == CargoVisionTarget.Color.RED) {
      return redCargo;
    }

    return blueCargo;
  }
}
