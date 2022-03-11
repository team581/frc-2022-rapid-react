// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.vision.VisionSubsystemBase;
import frc.robot.vision.VisionTarget;

/**
 * A command that sets the {@link VisionSubsystemBase vision system} to enable vision processing for
 * the provided {@link VisionTarget vision target}.
 */
public class UseVisionTargetCommand extends InstantCommand {
  private final VisionSubsystemBase visionSubsystem;
  private final VisionTarget visionTarget;

  public UseVisionTargetCommand(VisionSubsystemBase visionSubsystem, VisionTarget visionTarget) {
    addRequirements(visionSubsystem);

    this.visionSubsystem = visionSubsystem;
    this.visionTarget = visionTarget;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionSubsystem.useVisionTarget(visionTarget);
  }
}
