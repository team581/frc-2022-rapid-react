// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.vision.targets.VisionTarget;

public class WaitForVisionTargetCommand extends CommandBase {
  private final VisionTarget visionTarget;
  private final LimelightSubsystem limelightSubsystem;

  /** Creates a new WaitForVisionTargetCommand. */
  public WaitForVisionTargetCommand(
      LimelightSubsystem limelightSubsystem, VisionTarget visionTarget) {
    addRequirements(limelightSubsystem);

    this.limelightSubsystem = limelightSubsystem;
    this.visionTarget = visionTarget;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limelightSubsystem.hasTargets();
  }
}
