// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.commands;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.vision.VisionSubsystemBase;

/**
 * A command that finishes when the vision system has identified a vision target.
 *
 * <p>You must select the correct vision pipeline before calling the command!
 */
public class WaitForVisionTargetCommand extends WaitUntilCommand {
  /** Creates a new WaitForVisionTargetCommand. */
  public WaitForVisionTargetCommand(VisionSubsystemBase visionSubsystem) {
    super(visionSubsystem::hasTargets);

    addRequirements(visionSubsystem);
  }
}
