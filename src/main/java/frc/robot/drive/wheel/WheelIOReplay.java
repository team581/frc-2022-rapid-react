// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.wheel;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class WheelIOReplay implements WheelIO {
  @Override
  public void updateInputs(Inputs inputs) {
    // Intentionally left empty
  }

  @Override
  public void setVoltage(double outputVolts) {
    // Intentionally left empty
  }

  @Override
  public void zeroEncoder() {
    // Intentionally left empty
  }

  @Override
  public MotorController getMotorController() {
    // Intentionally left empty
    return null;
  }
}
