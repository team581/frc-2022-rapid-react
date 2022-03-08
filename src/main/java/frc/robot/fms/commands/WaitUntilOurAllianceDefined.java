// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.fms.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.fms.FmsSubsystem;

/** Retrieves our alliance data from the FMS. */
public class WaitUntilOurAllianceDefined extends WaitUntilCommand {
  /** Creates a new WaitUntilOurAllianceDefined. */
  public WaitUntilOurAllianceDefined(FmsSubsystem fmsSubsystem) {
    super(() -> fmsSubsystem.getOurAlliance() != Alliance.Invalid);
  }
}
