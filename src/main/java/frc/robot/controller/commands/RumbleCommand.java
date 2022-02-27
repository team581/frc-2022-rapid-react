// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controller.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controller.util.RumblePattern;

public class RumbleCommand extends CommandBase {
  private final Timer timer = new Timer();
  /** The controller to rumble. */
  private final XboxController controller;
  /** The rumble pattern to rumble the controller with. */
  private final RumblePattern pattern;

  /** The current rumble value of the controller. */
  private double currentRumbleValue = 0;
  /** The number of times a rumble has occurred. */
  private double times = 0;

  /** Creates a new RumbleCommand with a given controller and rumble pattern. */
  public RumbleCommand(XboxController controller, RumblePattern pattern) {
    this.controller = controller;
    this.pattern = pattern;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isRumbling()) {
      if (timer.hasElapsed(pattern.duration)) {
        stopRumble();
        timer.reset();
        times++;
      }
    } else {
      if (timer.hasElapsed(pattern.delay)) {
        setRumble(pattern.strength);
        timer.reset();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stopRumble();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pattern.times == -1) {
      return false;
    }

    return times >= pattern.times;
  }

  private boolean isRumbling() {
    return currentRumbleValue == 0;
  }

  private void setRumble(double value) {
    if (value == currentRumbleValue) {
      // Avoid unecessarily setting the rumble value
      return;
    }

    currentRumbleValue = value;

    controller.setRumble(RumbleType.kLeftRumble, currentRumbleValue);
    controller.setRumble(RumbleType.kRightRumble, currentRumbleValue);
  }

  private void stopRumble() {
    setRumble(0);
  }
}
