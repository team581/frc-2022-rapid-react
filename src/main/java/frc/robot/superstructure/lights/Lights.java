// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.lights;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.superstructure.arm.Arm;
import frc.robot.superstructure.arm.ArmPosition;
import frc.robot.superstructure.lights.LightsIO.Inputs;
import frc.robot.superstructure.swiffer.Swiffer;
import frc.robot.superstructure.swiffer.SwifferMode;
import org.littletonrobotics.junction.Logger;

public class Lights extends SubsystemBase {
  /** The duration (in seconds) that lights should be on or off when in fast blink mode. */
  private static final double FAST_BLINK_DURATION = 0.08;

  /** The duration (in seconds) that lights should be on or off when in slow blink mode. */
  private static final double SLOW_BLINK_DURATION = 0.25;

  private final Timer blinkTimer = new Timer();
  private final Inputs inputs = new Inputs();

  private LightsIO io;

  private SwifferMode swifferState = SwifferMode.STOPPED;
  private boolean swifferAtGoal = true;

  private ArmPosition armState = ArmPosition.UP;
  private boolean armAtGoal = true;
  private Color currentColor = Color.kBlack;

  /** Creates a new Lights. */
  public Lights(LightsIO io) {
    this.io = io;

    blinkTimer.reset();
    blinkTimer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Lights", inputs);

    if (DriverStation.isEnabled()) {
      chooseColor();
    } else {
      setColor(Color.kBlue, LightsMode.SOLID);
    }

    // Flushing color is required every tick for the blinking modes
    io.flushColor();
  }

  /** Sets the {@link Swiffer swiffer}'s state. */
  public void setSubsystemState(SwifferMode swifferState, boolean atGoal) {
    this.swifferState = swifferState;
    this.swifferAtGoal = atGoal;
  }

  /** Sets the {@link Arm arm}'s state. */
  public void setSubsystemState(ArmPosition armState, boolean atGoal) {
    this.armState = armState;
    this.armAtGoal = atGoal;
  }

  private void chooseColor() {
    Color color;
    LightsMode lightsMode;

    switch (armState) {
      case UP:
        switch (swifferState) {
          case STOPPED:
            color = Color.kYellow;
            lightsMode = armAtGoal && swifferAtGoal ? LightsMode.BLINK_SLOW : LightsMode.BLINK_FAST;
            break;
          case SHOOTING:
            color = Color.kRed;
            lightsMode = armAtGoal ? LightsMode.BLINK_FAST : LightsMode.BLINK_SLOW;
            break;
          default:
            // Should never happen
            setColorForError();
            return;
        }
        break;
      case DOWN:
        color = Color.kGreen;
        lightsMode = armAtGoal && swifferAtGoal ? LightsMode.BLINK_FAST : LightsMode.BLINK_SLOW;
        break;
      default:
        // Should never happen
        setColorForError();
        return;
    }

    setColor(color, lightsMode);

    Logger.getInstance()
        .recordOutput("Lights/Color", new double[] {color.red, color.green, color.blue});
    Logger.getInstance().recordOutput("Lights/Mode", lightsMode.toString());
  }

  private void setColor(Color color, LightsMode mode) {
    switch (mode) {
      case SOLID:
        setColorSolid(color);
        break;
      case BLINK_FAST:
        setColorBlink(color, FAST_BLINK_DURATION);
        break;
      case BLINK_SLOW:
        setColorBlink(color, SLOW_BLINK_DURATION);
        break;
    }

    currentColor = color;
  }

  private void setColorSolid(Color color) {
    for (var i = 0; i < inputs.ledCount; i++) {
      io.setColor(i, color);
    }
  }

  private void setColorBlink(Color color, double duration) {
    if (color != currentColor) {
      // Force the new color to appear immediately instead of having a brief delay where the lights
      // are turned off
      setColorSolid(color);
      return;
    }

    final var time = blinkTimer.get();

    if (time >= duration) {
      if (time >= duration * 2) {
        blinkTimer.reset();
        setColorSolid(Color.kBlack);
      } else {
        setColorSolid(color);
      }
    }
  }

  /** Sets the lights to the color pattern for when an error has occurred. */
  private void setColorForError() {
    setColor(Color.kPurple, LightsMode.BLINK_FAST);
  }
}
