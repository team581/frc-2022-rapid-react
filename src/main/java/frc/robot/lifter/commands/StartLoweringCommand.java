// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lifter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lifter.LifterSubsystem;

/** Start lowering and knowing when to stop at the bottom */
public class StartLoweringCommand extends CommandBase {
  private final LifterSubsystem lifter;

  /** Creates a new StartLoweringCommand. */
  public StartLoweringCommand(LifterSubsystem lifter) {
    this.lifter = lifter;
    addRequirements(lifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lifter.startLowering();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // once at the end just stop
    lifter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // finished once the lifter is fully down
    return lifter.getPosition() == LifterSubsystem.Position.DOWN;
  }
}
