// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_cargo;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.misc.util.PolarPose2d;
import frc.robot.vision.VisionSubsystemBase;
import frc.robot.vision_cargo.CargoVisionTarget.Color;
import frc.robot.vision_upper.TimestampedPose2d;
import java.util.Optional;

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
        HEIGHT_FROM_FLOOR = Units.inchesToMeters(15.5);
        ANGLE_OF_ELEVATION = new Rotation2d(0);
        break;
      default:
        // 'Cause the test data was made when the robot was up high
        HEIGHT_FROM_FLOOR = Units.feetToMeters(4);
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
      System.out.println("no camera to hub translation");
      return Optional.empty();
    }
    System.out.println("yes camera to hub translation");

    final var cameraToHub = optionalCameraToHub.get();
    // Apply the camera's rotational error to the robot's heading
    final var adjustedAngle = imu.getRotation().minus(cameraToHub.getTheta());

    // Get the polar coordinate of the target
    final var robotToHubPolar = new PolarPose2d(cameraToHub.getR(), adjustedAngle);

    // Translate to Cartesian coordinates – this is the estimate relative position of the Hub,
    // relative to the camera
    final var offsetToHubFromRobot = robotToHubPolar.getTranslation2d();

    // Let's start with the Hub's position
    // Not sure where to put Hub-related functions -- is there already one to ask for the Hub's
    // position?
    // Gonna brute force it here.
    final var hubTranslation =
        new Translation2d(Constants.FIELD_WIDTH / 2.0, Constants.FIELD_LENGTH / 2.0);
    // Now subtract the translation from the camera to the Hub. We subtract because we're going back
    // towards the camera from what it saw.
    final var robotTranslation = hubTranslation.minus(offsetToHubFromRobot);

    // Need to use whatever the robot's facing was when the vision target was seen.
    // TODO: Keep an interpolated tree map of IMU rotation histories to get the robot heading at image capture
    final var robotHeading = imu.getRotation();

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
