// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LightsIORoborio implements LightsIO {
  private final AddressableLED leds = new AddressableLED(9);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(8);

  public LightsIORoborio() {
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
