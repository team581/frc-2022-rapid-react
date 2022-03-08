// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelight_cargo.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.fms.FmsSubsystem;
import frc.robot.fms.commands.WaitUntilOurAllianceDefined;
import frc.robot.limelight_cargo.CargoLimelightSubsystem;

/** Sets the alliance colors for opponent and our cargo. */
public class SetCargoColorsCommand extends SequentialCommandGroup {
  /** Creates a new SetCargoColorsCommand. */
  public SetCargoColorsCommand(
      CargoLimelightSubsystem cargoLimelightSubsystem, FmsSubsystem fmsSubsystem) {
    addCommands(
        // Wait until our alliance is known
        new WaitUntilOurAllianceDefined(fmsSubsystem),
        // Update the state in the subsystem
        new InstantCommand(
            () -> cargoLimelightSubsystem.setAlliances(fmsSubsystem.getOurAlliance())));
  }
}
