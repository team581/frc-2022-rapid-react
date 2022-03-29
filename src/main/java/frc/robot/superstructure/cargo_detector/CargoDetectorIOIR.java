// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.cargo_detector;

import edu.wpi.first.wpilibj.DigitalInput;

public class CargoDetectorIOIR implements CargoDetectorIO {
  private final DigitalInput leftSensor = new DigitalInput(0);
  private final DigitalInput rightSensor = new DigitalInput(1);

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.hasLeftCargo = leftSensor.get();
    inputs.hasRightCargo = rightSensor.get();
  }
}
