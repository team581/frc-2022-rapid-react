// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_cargo.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.match_metadata.MatchMetadataSubsystem;
import frc.robot.match_metadata.commands.WaitUntilOurAllianceDefined;
import frc.robot.vision_cargo.CargoVisionSubsystem;

/** Sets the alliance colors for opponent and our cargo. */
public class SetCargoColorsCommand extends SequentialCommandGroup {
  /** Creates a new SetCargoColorsCommand. */
  public SetCargoColorsCommand(
      CargoVisionSubsystem cargoVisionSubsystem, MatchMetadataSubsystem matchMetadataSubsystem) {
    addCommands(
        // Wait until our alliance is known
        new WaitUntilOurAllianceDefined(matchMetadataSubsystem),
        // Update the state in the subsystem
        new InstantCommand(
            () -> cargoVisionSubsystem.setAlliances(matchMetadataSubsystem.getOurAlliance())));
  }
}
