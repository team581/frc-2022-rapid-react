// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import lib.limelight.Limelight;
import lib.limelight.Limelight.CamMode;
import lib.limelight.Limelight.StreamingMode;
import org.littletonrobotics.junction.Logger;

public abstract class VisionIOLimelight implements VisionIO {
  protected Rotation2d angleOfElevation;
  protected double heightFromFloor;

  /** The number of seconds it takes to capture an image. */
  private static final double IMAGE_CAPTURE_LATENCY = Units.millisecondsToSeconds(11);

  private final Limelight limelight;

  protected VisionIOLimelight(Limelight limelight) {
    this.limelight = limelight;
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.captureTimestamp =
        Logger.getInstance().getTimestamp()
            - Units.millisecondsToSeconds(limelight.getPipelineLatency())
            - IMAGE_CAPTURE_LATENCY;
    inputs.hasTargets = limelight.hasTargets();
    inputs.tx = limelight.getX();
    inputs.ty = limelight.getY();
    inputs.corners = Inputs.coordinateArrayToTranslation2dList(limelight.getCorners());
  }

  @Override
  public void setCamMode(CamMode camMode) {
    limelight.setCamMode(camMode);
  }

  @Override
  public void setPipeline(int pipeline) {
    limelight.setPipeline(pipeline);
  }

  @Override
  public void setStreamingMode(StreamingMode streamingMode) {
    limelight.setStreamingMode(streamingMode);
  }
}
