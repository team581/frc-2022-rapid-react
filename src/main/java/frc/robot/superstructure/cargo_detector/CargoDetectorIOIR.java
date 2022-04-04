// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.cargo_detector;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.misc.exceptions.UnsupportedSubsystemException;

public class CargoDetectorIOIR implements CargoDetectorIO {
  private final DigitalInput leftSensor;
  private final DigitalInput rightSensor;

  public CargoDetectorIOIR() {
    switch (Constants.getRobot()) {
      case TEST_2020_BOT:
        leftSensor = new DigitalInput(9);
        // TODO: Wire this sensor
        rightSensor = new DigitalInput(25);
        break;
      case SIM_BOT:
        leftSensor = new DigitalInput(9);
        rightSensor = new DigitalInput(8);
        break;
      default:
        throw new UnsupportedSubsystemException(this);
    }
  }

  @Override
  public void updateInputs(Inputs inputs) {
    // The sensor outputs an off signal when it triggers, so we invert the return value
    inputs.hasLeftCargo = !leftSensor.get();
    inputs.hasRightCargo = !rightSensor.get();
  }
}
