// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_upper;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.vision.VisionSubsystemBase;
import frc.robot.vision_cargo.UpperHubVisionTarget;
import org.littletonrobotics.junction.Logger;

public class UpperHubVisionSubsystem extends VisionSubsystemBase {
  private static final String LOGGER_NAME = "UpperVision";

  public enum Pipelines {
    UPPER_HUB(0),
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
      default:
        HEIGHT_FROM_FLOOR = 0.6;
        ANGLE_OF_ELEVATION = new Rotation2d(0);
        break;
    }
  }

  private final ImuSubsystem imu;

  /** Creates a new UpperHubVisionSubsystem. */
  public UpperHubVisionSubsystem(UpperHubVisionIO io, ImuSubsystem imu) {
    super(LOGGER_NAME, io, Pipelines.DRIVER_MODE.index);

    this.imu = imu;
  }

  @Override
  public void periodic() {
    super.periodic();

    if (hasTargets()) {
      Logger.getInstance()
          .recordOutput(
              LOGGER_NAME + "/VisionTarget",
              new double[] {UpperHubVisionTarget.POSE.getX(), UpperHubVisionTarget.POSE.getY()});
    }
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
