// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_upper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.misc.util.LoggingUtil;
import frc.robot.vision.Camera;
import frc.robot.vision.ComputerVisionUtilForCamera;
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

  /** Creates a new UpperHubVisionSubsystem. */
  public UpperHubVisionSubsystem(UpperHubVisionIO io) {
    super(LOGGER_NAME, io, Pipelines.DRIVER_MODE.index);
  }

  @Override
  public void periodic() {
    super.periodic();

    if (hasTargets()) {
      Logger.getInstance()
          .recordOutput(
              LOGGER_NAME + "/VisionTarget",
              LoggingUtil.translationToArray(UpperHubVisionTarget.COORDINATES));
    }
  }
}
