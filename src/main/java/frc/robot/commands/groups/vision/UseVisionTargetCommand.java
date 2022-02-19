// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LimelightSubsystemBase;
import frc.robot.vision.targets.LimelightVisionTarget;

/**
 * A command that sets the {@link LimelightSubsystemBase Limelight} to enable vision processing for
 * the provided {@link LimelightVisionTarget vision target}.
 */
public class UseVisionTargetCommand extends InstantCommand {
  private final LimelightSubsystemBase limelightSubsystem;
  private final LimelightVisionTarget visionTarget;

  public UseVisionTargetCommand(
      LimelightSubsystemBase limelightSubsystem, LimelightVisionTarget visionTarget) {
    addRequirements(limelightSubsystem);

    this.limelightSubsystem = limelightSubsystem;
    this.visionTarget = visionTarget;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelightSubsystem.useVisionTarget(visionTarget);
  }
}
