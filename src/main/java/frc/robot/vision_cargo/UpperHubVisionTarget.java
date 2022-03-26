// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_cargo;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.vision.VisionTarget;

/** The upper hub ring vision target. */
public class UpperHubVisionTarget extends VisionTarget {
  public static final Translation2d POSE =
      new Translation2d(Constants.FIELD_WIDTH / 2, Constants.FIELD_LENGTH / 2);

  public UpperHubVisionTarget(CargoVisionSubsystem vision) {
    super(
        vision,
        // TODO: See where the camera is placing the crosshair on the target
        Units.feetToMeters(8) + Units.inchesToMeters(5.0 + (5.0 / 8.0)),
        CargoVisionSubsystem.Pipelines.UPPER_HUB.index);
  }
}
