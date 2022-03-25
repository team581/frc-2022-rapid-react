// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.cargo_detector;

import edu.wpi.first.wpilibj.AnalogInput;

public class CargoDetectorIOIR implements CargoDetectorIO {
  private final AnalogInput sensor = new AnalogInput(0);

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.distance = sensor.getValue();
  }
}
