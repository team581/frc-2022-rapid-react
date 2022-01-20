// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * A command that finishes when the Limelight has identified a vision target.
 *
 * <p>You must select the correct Limelight pipeline before calling the command!
 */
public class WaitForVisionTargetCommand extends CommandBase {
  private final LimelightSubsystem limelight;

  /** Creates a new WaitForVisionTargetCommand. */
  public WaitForVisionTargetCommand(LimelightSubsystem limelight) {
    addRequirements(limelight);

    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limelight.hasTargets();
  }
}
