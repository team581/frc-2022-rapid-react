// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.misc.exceptions.UnknownTargetRobotException;

public class LightsIORoborio implements LightsIO {
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;

  public LightsIORoborio() {
    switch (Constants.getRobot()) {
      case SIM_BOT:
        leds = new AddressableLED(9);
        buffer = new AddressableLEDBuffer(8 * 6);
        break;
      case TEST_2020_BOT:
        leds = new AddressableLED(9);
        buffer = new AddressableLEDBuffer(8);
        break;
      default:
        throw new UnknownTargetRobotException();
    }

    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void setColor(int index, Color color) {
    buffer.setLED(index, color);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.ledCount = buffer.getLength();
  }

  @Override
  public void flushColor() {
    leds.setData(buffer);
  }
}
