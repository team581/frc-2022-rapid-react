// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

public class WheelIOReplay extends WheelIOSim implements WheelIO {
  public WheelIOReplay(Corner corner) {
    super(corner);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    // Intentionally left empty
  }

  @Override
  public void setVoltage(double outputVolts) {
    // Intentionally left empty
  }
}
