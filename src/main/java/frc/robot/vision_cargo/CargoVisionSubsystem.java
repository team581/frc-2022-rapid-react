// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_cargo;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.misc.util.PolarTranslation2d;
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

  private static final double HEIGHT_FROM_FLOOR;
  private static final Rotation2d ANGLE_OF_ELEVATION;

  static {
    switch (Constants.getRobot()) {
      case TEST_2020_BOT:
      case SIM_BOT:
        HEIGHT_FROM_FLOOR = Units.inchesToMeters(15.5);
        ANGLE_OF_ELEVATION = new Rotation2d(0);
        break;
      default:
        HEIGHT_FROM_FLOOR = Units.feetToMeters(1);
        ANGLE_OF_ELEVATION = new Rotation2d(0);
        break;
    }
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

      final var visionPose = getRobotPose().orElseThrow();
      Logger.getInstance()
          .recordOutput(
              loggerName + "/RobotPose",
              new double[] {
                visionPose.pose.getX(),
                visionPose.pose.getY(),
                visionPose.pose.getRotation().getRadians()
              });
      Logger.getInstance()
          .recordOutput(
              loggerName + "/VisionTargetPose",
              new double[] {UpperHubVisionTarget.POSE.getX(), UpperHubVisionTarget.POSE.getY()});
    }
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

    final var cameraToHub = optionalCameraToHub.get();

    // Need to use whatever the robot's facing was when the vision target was seen.
    // TODO: Keep an interpolated tree map of IMU rotation histories to get the robot heading at
    // image capture
    // TODO: This should maybe be .plus(). We tested using an inverted gyroscope heading and so the
    // angle related parts of this code are probably wrong.
    final var robotHeading = imu.getRotation().minus(cameraToHub.getTheta());

    // Get the polar coordinate of the target
    final var robotToHubPolar = new PolarTranslation2d(cameraToHub.getR(), robotHeading);

    // Translate to Cartesian coordinates - this is the estimate relative position of the Hub,
    // relative to the camera
    final var offsetToHubFromRobot = robotToHubPolar.getTranslation2d();

    // Let's start with the Hub's position
    // Now subtract the translation from the camera to the Hub. We subtract because we're going back
    // towards the camera from what it saw.
    final var robotTranslation = UpperHubVisionTarget.POSE.minus(offsetToHubFromRobot);

    final var robotPose = new Pose2d(robotTranslation, robotHeading);

    return Optional.of(new TimestampedPose2d(robotPose, inputs.captureTimestamp));
  }

  /** Gets the {@link CargoVisionTarget} instance for the provided alliance (red or blue). */
  private CargoVisionTarget getTargetForAlliance(CargoVisionTarget.Color alliance) {
    if (alliance == CargoVisionTarget.Color.RED) {
      return redCargo;
    }

    return blueCargo;
  }

  @Override
  protected Rotation2d getAngleOfElevation() {
    return ANGLE_OF_ELEVATION;
  }

  @Override
  protected double getHeightFromFloor() {
    return HEIGHT_FROM_FLOOR;
  }
}
