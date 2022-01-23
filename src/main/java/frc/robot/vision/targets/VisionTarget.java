// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

/** A class to represent a vision target on the field. */
public abstract class VisionTarget {
  /**
   * Prepares this vision target for use. This is where you should select the correct pipeline,
   * enable LEDs, etc.
   *
   * <p>You must call this function before trying to use a vision system to track a vision target.
   */
  public abstract void prepareForUse();
}
