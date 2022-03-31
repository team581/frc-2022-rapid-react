// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

/** A vision target for the vision system. */
public abstract class VisionTarget {
  /** The index of this vision target's pipeline. */
  public final int pipeline;

  /**
   * The height from the floor to this vision target, in meters.
   *
   * @see
   *     <p>The <code>h2</code> distance in this diagram
   *     https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   */
  public final double heightFromFloor;

  protected VisionTarget(double heightFromFloor, int pipeline) {
    this.heightFromFloor = heightFromFloor;
    this.pipeline = pipeline;
  }
}
