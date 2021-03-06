// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

// Talon FX already has a good built-in simulation system, so we can just extend the Falcon 500 IO
// class
public class WheelIOSim extends WheelIOFalcon500 implements WheelIO {
  public WheelIOSim(Corner corner) {
    super(corner);
  }
}
