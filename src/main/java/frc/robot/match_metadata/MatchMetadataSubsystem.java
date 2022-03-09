// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.match_metadata;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.match_metadata.MatchMetadataIO.Inputs;
import org.littletonrobotics.junction.Logger;

/** Subsystem for communicating with the driver station and FMS to get match metadata. */
public class MatchMetadataSubsystem extends SubsystemBase {
  private final MatchMetadataIO io;
  private final Inputs inputs = new Inputs();

  /** Creates a new MatchMetadataSubsystem. */
  public MatchMetadataSubsystem(MatchMetadataIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("MatchMetadata", inputs);
  }

  /** Get our team's alliance. */
  public Alliance getOurAlliance() {
    return inputs.alliance;
  }
}
