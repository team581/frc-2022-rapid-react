// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.util.Units;
import lib.limelight.Limelight;
import org.littletonrobotics.junction.Logger;

public abstract class LimelightVisionIO implements VisionIO {
  /** The number of seconds it takes to capture an image. */
  private static final double IMAGE_CAPTURE_LATENCY = Units.millisecondsToSeconds(11);

  private final Limelight limelight;

  protected LimelightVisionIO(Limelight limelight) {
    this.limelight = limelight;
  }

  @Override
  public void updateInputs(Inputs inputs) {
    /** The total amount of time (in seconds) from when the image was captured to now. */
    final var totalCameraLatency =
        Units.millisecondsToSeconds(limelight.getPipelineLatency()) + IMAGE_CAPTURE_LATENCY;

    inputs.captureTimestamp = Logger.getInstance().getRealTimestamp() - totalCameraLatency;
    inputs.hasTargets = limelight.hasTargets();
    inputs.tx = limelight.getX();
    inputs.ty = limelight.getY();
    inputs.corners = Inputs.coordinateArrayToTranslation2dList(limelight.getCorners());
  }
}
