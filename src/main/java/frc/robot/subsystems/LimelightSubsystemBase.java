// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.groups.vision.UseDriverModeCommand;
import frc.robot.vision.targets.LimelightVisionTarget;
import lib.limelight.Limelight;

public abstract class LimelightSubsystemBase extends SubsystemBase {
  public final Limelight limelight;

  /**
   * The angle of elevation of this Limelight, in radians.
   *
   * @see
   *     <p>The <code>a1</code> angle in this diagram
   *     https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   */
  public final double angleOfElevation;
  /**
   * The height from the floor to this Limelight, in meters.
   *
   * @see
   *     <p>The <code>h1</code> distance in this diagram
   *     https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   */
  public final double heightFromFloor;

  private final int driverModePipeline;
  private boolean isDriverMode = false;

  /**
   * Creates a new LimelightSubsystemBase.
   *
   * @param name The NetworkTables name of this Limelight
   * @param angleOfElevation The Limelight's angle of elevation, in radians
   * @param heightFromFloor The Limelight's height from the floor, in meters
   * @param driverModePipeline The index of the pipeline to use when in driver mode
   */
  protected LimelightSubsystemBase(
      String name, double angleOfElevation, double heightFromFloor, int driverModePipeline) {
    this.limelight = new Limelight(name);
    this.angleOfElevation = angleOfElevation;
    this.heightFromFloor = heightFromFloor;
    this.driverModePipeline = driverModePipeline;

    // Enable driver mode when other commands aren't using vision processing
    setDefaultCommand(new UseDriverModeCommand(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Whether this Limelight is in driver mode. */
  public boolean isDriverMode() {
    return isDriverMode;
  }

  /**
   * Enables raw camera output and disables computer processing.
   *
   * @see {@link frc.robot.commands.groups.vision.UseDriverModeCommand}
   */
  public void useDriverMode() {
    isDriverMode = true;
    limelight.setCamMode(Limelight.CamMode.DRIVER_CAMERA);
    limelight.setStreamingMode(Limelight.StreamingMode.PIP_MAIN);
    limelight.setPipeline(driverModePipeline);
  }

  /**
   * Enables vision processing for the provided vision target.
   *
   * @see {@link frc.robot.commands.groups.vision.UseVisionTargetCommand}
   */
  public void useVisionTarget(LimelightVisionTarget target) {
    isDriverMode = false;
    limelight.setCamMode(Limelight.CamMode.VISION_PROCESSOR);
    limelight.setStreamingMode(Limelight.StreamingMode.PIP_SECONDARY);
    limelight.setPipeline(target.pipeline);
  }
}
