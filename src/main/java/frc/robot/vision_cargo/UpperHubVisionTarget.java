// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_cargo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.misc.util.PolarTranslation2d;
import frc.robot.vision.VisionTarget;

/** The upper hub ring vision target. */
public class UpperHubVisionTarget extends VisionTarget {
  public static final Translation2d COORDINATES =
      new Translation2d(Constants.FIELD_WIDTH / 2, Constants.FIELD_LENGTH / 2);

  public static final double RADIUS = Units.feetToMeters(4) / 2;

  /**
   * The translation from the outer ring of the vision target to the center of the hub structure.
   */
  public static final PolarTranslation2d TRANSLATION_FROM_OUTER_RING_TO_CENTER =
      new PolarTranslation2d(RADIUS, new Rotation2d());

  public UpperHubVisionTarget() {
    // Measurements from https://firstfrc.blob.core.windows.net/frc2022/Manual/2022FRCGameManual.pdf
    super(
        // The vision tape is 8 feet, 5 and 5/8 inches from the carpet
        Units.feetToMeters(8)
            + Units.inchesToMeters(5)
            + Units.inchesToMeters(5.0 / 8.0)
            // The crosshair is placed in the center of the vision tape which is 2 inches tall
            + Units.inchesToMeters(2) / 2,
        CargoVisionSubsystem.Pipelines.UPPER_HUB.index);
  }
}
