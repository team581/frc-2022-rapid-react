// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.limelight_cargo.CargoLimelightSubsystem;
import frc.robot.limelight_cargo.CargoVisionTarget.Color;

/** Retrieves our alliance data from the FMS. */
public class RefreshAllianceWithFmsCommand extends CommandBase {
  private final CargoLimelightSubsystem limelightSubsystem;
  private Alliance alliance = Alliance.Invalid;

  /** Creates a new SetAllianceCommand. */
  public RefreshAllianceWithFmsCommand(CargoLimelightSubsystem cargoLimelightSubsystem) {
    this.limelightSubsystem = cargoLimelightSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    alliance = DriverStation.getAlliance();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (alliance == Alliance.Red) {
      this.limelightSubsystem.setOurAlliance(Color.RED);
    } else if (alliance == Alliance.Blue) {
      this.limelightSubsystem.setOurAlliance(Color.BLUE);
    }

    // Do nothing otherwise, maybe the command was interrupted because no alliance was found in time
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return alliance != Alliance.Invalid;
  }
}
