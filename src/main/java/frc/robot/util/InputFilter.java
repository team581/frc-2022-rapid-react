// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.subsystems.LimelightSubsystem;

/**
 * A class for filtering joystick input from the driver.
 *
 * <p>Mostly just a container for managing who is currently controlling the robot, the human driver
 * or the autonomous computer system.
 */
public class InputFilter {
  private boolean ignoreJoysticks = false;
  private LimelightSubsystem limelightSubsystem;

  public InputFilter(LimelightSubsystem limelightSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
  }

  public void useDriverControl() {
    ignoreJoysticks = false;
    this.limelightSubsystem.useDriverMode();
  }

  public void useComputerControl() {
    ignoreJoysticks = true;
  }

  public boolean shouldIgnoreJoysticks() {
    return ignoreJoysticks;
  }
}
