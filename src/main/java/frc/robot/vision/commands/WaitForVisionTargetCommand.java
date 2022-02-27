// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.commands;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.vision.LimelightSubsystemBase;

/**
 * A command that finishes when the Limelight has identified a vision target.
 *
 * <p>You must select the correct Limelight pipeline before calling the command!
 */
public class WaitForVisionTargetCommand extends WaitUntilCommand {
  /** Creates a new WaitForVisionTargetCommand. */
  public WaitForVisionTargetCommand(LimelightSubsystemBase limelightSubsystem) {
    super(limelightSubsystem.limelight::hasTargets);

    addRequirements(limelightSubsystem);
  }
}
