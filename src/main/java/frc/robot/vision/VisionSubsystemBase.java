// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.VisionIO.Inputs;
import frc.robot.vision.commands.UseDriverModeCommand;
import lib.limelight.Limelight;
import org.littletonrobotics.junction.Logger;

public abstract class VisionSubsystemBase extends SubsystemBase {
  private final int driverModePipeline;
  private boolean isDriverMode = false;

  protected final VisionIO io;
  protected final Inputs inputs = new Inputs();
  protected final String loggerName;

  /**
   * Creates a new VisionSubsystemBase.
   *
   * @param loggerName The name to use in the logger
   * @param io The IO layer to use
   * @param driverModePipeline The index of the pipeline to use when in driver mode
   */
  protected VisionSubsystemBase(String loggerName, VisionIO io, int driverModePipeline) {
    this.loggerName = loggerName;
    this.io = io;
    this.driverModePipeline = driverModePipeline;

    // Enable driver mode when other commands aren't using vision processing
    setDefaultCommand(
        new UseDriverModeCommand(this)
            .perpetually()
            .withName("Perpetual" + UseDriverModeCommand.class.getSimpleName()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.getInstance().processInputs(loggerName, inputs);
    Logger.getInstance().recordOutput(loggerName + "/CornerCount", inputs.corners.size());
  }

  /** Whether this camera is in driver mode with vision processing disabled. */
  public boolean isDriverMode() {
    return isDriverMode;
  }

  /**
   * Enables raw camera output and disables computer processing.
   *
   * @see {@link frc.robot.vision.commands.UseDriverModeCommand}
   */
  public void useDriverMode() {
    // isDriverMode = true;
    // io.setCamMode(Limelight.CamMode.DRIVER_CAMERA);
    // io.setStreamingMode(Limelight.StreamingMode.PIP_MAIN);
    // io.setPipeline(driverModePipeline);
  }

  /**
   * Enables vision processing for the provided vision target.
   *
   * @see {@link frc.robot.vision.commands.UseVisionTargetCommand}
   */
  public void useVisionTarget(VisionTarget target) {
    isDriverMode = false;
    io.setCamMode(Limelight.CamMode.VISION_PROCESSOR);
    io.setStreamingMode(Limelight.StreamingMode.PIP_SECONDARY);
    io.setPipeline(target.pipeline);
  }

  public Rotation2d getX() {
    return inputs.tx;
  }

  public Rotation2d getY() {
    return inputs.ty;
  }

  public boolean hasTargets() {
    return inputs.hasTargets;
  }
}
