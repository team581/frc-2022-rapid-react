// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swiffer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swiffer.SwifferIO.Inputs;
import org.littletonrobotics.junction.Logger;

public class SwifferSubsystem extends SubsystemBase {
  private final SwifferIO io;
  private final Inputs inputs = new Inputs();

  /** Creates a new SwifferSubsystem. */
  public SwifferSubsystem(SwifferIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Swiffer", inputs);
  }

  public void startSnarfing() {
    // TODO: Tune this value
    io.setMotorPercentage(0.1);
  }

  public void startShooting() {
    // TODO: Tune this value
    io.setMotorPercentage(-0.1);
  }

  public void stop() {
    io.setMotorPercentage(0);
  }
}
