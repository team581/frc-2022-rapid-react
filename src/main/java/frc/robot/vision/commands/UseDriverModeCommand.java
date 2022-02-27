// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.vision.LimelightSubsystemBase;

/** A command that sets the {@link LimelightSubsystemBase Limelight} to use driver mode. */
public class UseDriverModeCommand extends InstantCommand {
  private final LimelightSubsystemBase limelightSubsystem;

  public UseDriverModeCommand(LimelightSubsystemBase limelightSubsystem) {
    addRequirements(limelightSubsystem);

    this.limelightSubsystem = limelightSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelightSubsystem.useDriverMode();
  }
}
