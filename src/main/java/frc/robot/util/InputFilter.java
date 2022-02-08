// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.subsystems.CargoLimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystemBase;
import frc.robot.subsystems.UpperHubLimelightSubsystem;

/**
 * A class for filtering joystick input from the driver.
 *
 * <p>Mostly just a container for managing who is currently controlling the robot, the human driver
 * or the autonomous computer system.
 */
public class InputFilter {
  private final UpperHubLimelightSubsystem upperHubLimelightSubsystem;
  private final LimelightSubsystemBase cargoLimelightSubsystem;

  public InputFilter(
      UpperHubLimelightSubsystem upperHubLimelightSubsystem,
      CargoLimelightSubsystem cargoLimelightSubsystem) {
    this.upperHubLimelightSubsystem = upperHubLimelightSubsystem;
    this.cargoLimelightSubsystem = cargoLimelightSubsystem;
  }

  public void useDriverControl() {
    this.upperHubLimelightSubsystem.useDriverMode();
    this.cargoLimelightSubsystem.useDriverMode();
  }

  public void useCargoControl() {
    this.upperHubLimelightSubsystem.useDriverMode();
  }

  public void useUpperHubControl() {
    this.cargoLimelightSubsystem.useDriverMode();
  }

  public boolean shouldIgnoreJoysticks() {
    return !this.cargoLimelightSubsystem.isDriverMode()
        && !this.upperHubLimelightSubsystem.isDriverMode();
  }
}
