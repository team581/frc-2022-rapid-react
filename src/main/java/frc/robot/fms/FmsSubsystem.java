// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.fms;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.fms.FmsIO.Inputs;
import org.littletonrobotics.junction.Logger;

/** Subsystem for communicating with the driver station and FMS. */
public class FmsSubsystem extends SubsystemBase {
  private final FmsIO io;
  private final Inputs inputs = new Inputs();

  /** Creates a new FmsSubsystem. */
  public FmsSubsystem(FmsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Fms", inputs);
  }

  /** Get our team's alliance. */
  public Alliance getOurAlliance() {
    return inputs.alliance;
  }
}
