// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lifter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lifter.LifterIO.Inputs;
import org.littletonrobotics.junction.Logger;

public class LifterSubsystem extends SubsystemBase {
  public enum Position {
    UP,
    MIDDLE,
    DOWN,
    INVALID
  }

  private final LifterIO io;
  private final Inputs inputs = new Inputs();

  /** Creates a new LifterSubsystem. */
  public LifterSubsystem(LifterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Lifter", inputs);
    Logger.getInstance().recordOutput("Lifter/Position", getPosition().toString());
  }

  /** Gets the position of the lifter */
  public Position getPosition() {
    final var isUp = inputs.upperLimitSwitchActive;
    final var isDown = inputs.lowerLimitSwitchActive;

    // figuring out where the position is
    if (isUp) {
      if (isDown) {
        // Both sensors shoudln't be active at the same time
        return Position.INVALID;
      } else {
        return Position.UP;
      }
    }

    if (isDown) {
      return Position.DOWN;
    }

    return Position.MIDDLE;
  }

  public void startLifting() {
    // TODO: Tune this value
    io.setMotorPercentage(0.1);
  }

  public void startLowering() {
    // TODO: Tune this value
    io.setMotorPercentage(-0.1);
  }

  public void stop() {
    io.setMotorPercentage(0);
  }
}
