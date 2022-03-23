// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lights.LightsIO.Inputs;

public class LightsSubsystem extends SubsystemBase {
  private LightsIO io;
  private final Inputs inputs = new Inputs();

  /** Creates a new LightsSubsystem. */
  public LightsSubsystem(LightsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Lights", inputs);

    for (var i = 0; i < inputs.ledCount; i++) {
      io.setColor(i, Color.kRed);
    }
    io.flushColor();
  }
}
