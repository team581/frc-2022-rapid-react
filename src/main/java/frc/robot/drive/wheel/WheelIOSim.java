// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.wheel;

// Talon FX already has a good built-in simulation system, so we can just extend the real IO class
public class WheelIOSim extends WheelIOReal implements WheelIO {
  public WheelIOSim(Corner corner) {
    super(corner);
  }
}
