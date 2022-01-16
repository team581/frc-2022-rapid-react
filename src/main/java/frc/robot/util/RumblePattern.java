// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** A rumble pattern for a handheld controller. */
public class RumblePattern {
  /** The strength to rumble the controller with. */
  public final double strength;
  /** The time in seconds to rumble for. */
  public final double duration;
  /** The time in seconds to wait before rumbling again. */
  public final double delay;
  /**
   * The maximum number of times a rumble can occur before exiting. Use <code>-1</code> to rumble
   * indefinitely.
   */
  public final int times;

  /** Creates a new RumblePattern. */
  public RumblePattern(double strength, double duration, double delay, int times) {
    this.strength = strength;
    this.duration = duration;
    this.delay = delay;
    this.times = times;
  }

  /** Creates a new RumblePattern which rumbles an indefinite number of times. */
  public RumblePattern(double strength, double duration, double delay) {
    this(strength, duration, delay, -1);
  }
}
