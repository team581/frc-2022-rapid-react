// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

/** A class to represent a vision target on the field. */
public abstract class VisionTarget {
  /**
   * You should not call this function directly, instead use {@link
   * frc.robot.vision.Vision#useVisionTarget} to select a vision target for use.
   *
   * <p>A function that is called when this vision target is selected. This is where you should
   * select the correct pipeline, enable LEDs, etc.
   */
  public abstract void onSelected();
}
