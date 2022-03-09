// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.match_metadata;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MatchMetadataIOFms implements MatchMetadataIO {
  private boolean isAllianceInvalid = true;

  @Override
  public void updateInputs(Inputs inputs) {
    if (isAllianceInvalid) {
      // Only fetch from driver station if invalid
      final var alliance = DriverStation.getAlliance();

      inputs.alliance = alliance;

      isAllianceInvalid = alliance == Alliance.Invalid;
    }
  }
}
