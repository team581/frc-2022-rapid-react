// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.VisionIO.Inputs;
import frc.robot.vision.commands.UseDriverModeCommand;
import lib.limelight.Limelight;

public abstract class VisionSubsystemBase extends SubsystemBase {
  /**
   * The angle of elevation of the camera, in radians.
   *
   * @see
   *     <p>The <code>a1</code> angle in this diagram
   *     https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   */
  public final double angleOfElevation;
  // TODO: Consider refactoring this to be a Transform2d to the center of the robot
  /**
   * The height from the floor to the camera, in meters.
   *
   * @see
   *     <p>The <code>h1</code> distance in this diagram
   *     https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   */
  public final double heightFromFloor;

  private final int driverModePipeline;
  private boolean isDriverMode = false;

  private final VisionIO io;
  private final Inputs inputs = new Inputs();

  /**
   * Creates a new VisionSubsystemBase.
   *
   * @param io The IO layer to use
   * @param angleOfElevation The camera's angle of elevation, in radians
   * @param heightFromFloor The camera's height from the floor, in meters
   * @param driverModePipeline The index of the pipeline to use when in driver mode
   */
  protected VisionSubsystemBase(
      VisionIO io, double angleOfElevation, double heightFromFloor, int driverModePipeline) {
    this.io = io;
    this.angleOfElevation = angleOfElevation;
    this.heightFromFloor = heightFromFloor;
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
    isDriverMode = true;
    io.setCamMode(Limelight.CamMode.DRIVER_CAMERA);
    io.setStreamingMode(Limelight.StreamingMode.PIP_MAIN);
    io.setPipeline(driverModePipeline);
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

  public double getX() {
    return inputs.tx;
  }

  public double getY() {
    return inputs.ty;
  }

  public boolean hasTargets() {
    return inputs.hasTargets;
  }
}
