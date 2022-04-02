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
    super(
        // TODO: See where the camera is placing the crosshair on the target
        0.75, CargoVisionSubsystem.Pipelines.UPPER_HUB.index);
  }
}
