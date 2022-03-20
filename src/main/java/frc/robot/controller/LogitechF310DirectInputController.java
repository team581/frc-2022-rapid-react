// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controller;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Handle input from Logitech F310 (in DirectInput mode) controllers connected to the Driver
 * Station.
 *
 * <p>This class handles Logitech F310 input that comes from the Driver Station. Each time a value
 * is requested the most recent value is returned. There is a single class instance for each
 * controller and the mapping of ports to hardware buttons depends on the code in the Driver
 * Station.
 */
public class LogitechF310DirectInputController extends GenericHID {
  /** Represents a digital button on a LogitechF310DirectInputController. */
  public enum Button {
    LEFT_BUMPER(5),
    RIGHT_BUMPER(6),
    LEFT_STICK(11),
    RIGHT_STICK(12),
    LEFT_TRIGGER(7),
    RIGHT_TRIGGER(8),
    A_BUTTON(2),
    B_BUTTON(3),
    X_BUTTON(1),
    Y_BUTTON(4),
    BACK_BUTTON(9),
    START_BUTTON(10);

    public final int value;

    Button(int value) {
      this.value = value;
    }
  }

  /** Represents an axis on an LogitechF310DirectInputController. */
  public enum Axis {
    LEFT_X(0),
    RIGHT_X(2),
    LEFT_Y(1),
    RIGHT_Y(3);

    @SuppressWarnings("MemberName")
    public final int value;

    Axis(int value) {
      this.value = value;
    }
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public LogitechF310DirectInputController(final int port) {
    super(port);

    HAL.report(tResourceType.kResourceType_XboxController, port + 1);
  }

  /**
   * Get the X axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftX() {
    return getRawAxis(Axis.LEFT_X.value);
  }

  /**
   * Get the X axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightX() {
    return getRawAxis(Axis.RIGHT_X.value);
  }

  /**
   * Get the Y axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftY() {
    return getRawAxis(Axis.LEFT_Y.value);
  }

  /**
   * Get the Y axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightY() {
    return getRawAxis(Axis.RIGHT_Y.value);
  }

  /**
   * Read the value of the left bumper (LB) button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftBumper() {
    return getRawButton(Button.LEFT_BUMPER.value);
  }

  /**
   * Read the value of the right bumper (RB) button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightBumper() {
    return getRawButton(Button.RIGHT_BUMPER.value);
  }

  /**
   * Whether the left bumper (LB) was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftBumperPressed() {
    return getRawButtonPressed(Button.LEFT_BUMPER.value);
  }

  /**
   * Whether the right bumper (RB) was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightBumperPressed() {
    return getRawButtonPressed(Button.RIGHT_BUMPER.value);
  }

  /**
   * Whether the left bumper (LB) was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftBumperReleased() {
    return getRawButtonReleased(Button.LEFT_BUMPER.value);
  }

  /**
   * Whether the right bumper (RB) was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightBumperReleased() {
    return getRawButtonReleased(Button.RIGHT_BUMPER.value);
  }

  /**
   * Read the value of the left stick button (LSB) on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftStickButton() {
    return getRawButton(Button.LEFT_STICK.value);
  }

  /**
   * Read the value of the right stick button (RSB) on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightStickButton() {
    return getRawButton(Button.RIGHT_STICK.value);
  }

  /**
   * Whether the left stick button (LSB) was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftStickButtonPressed() {
    return getRawButtonPressed(Button.LEFT_STICK.value);
  }

  /**
   * Whether the right stick button (RSB) was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightStickButtonPressed() {
    return getRawButtonPressed(Button.RIGHT_STICK.value);
  }

  /**
   * Whether the left stick button (LSB) was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftStickButtonReleased() {
    return getRawButtonReleased(Button.LEFT_STICK.value);
  }

  /**
   * Whether the right stick (RSB) button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightStickButtonReleased() {
    return getRawButtonReleased(Button.RIGHT_STICK.value);
  }

  /**
   * Read the value of the A button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getAButton() {
    return getRawButton(Button.A_BUTTON.value);
  }

  /**
   * Whether the A button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getAButtonPressed() {
    return getRawButtonPressed(Button.A_BUTTON.value);
  }

  /**
   * Whether the A button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getAButtonReleased() {
    return getRawButtonReleased(Button.A_BUTTON.value);
  }

  /**
   * Read the value of the B button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getBButton() {
    return getRawButton(Button.B_BUTTON.value);
  }

  /**
   * Whether the B button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getBButtonPressed() {
    return getRawButtonPressed(Button.B_BUTTON.value);
  }

  /**
   * Whether the B button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getBButtonReleased() {
    return getRawButtonReleased(Button.B_BUTTON.value);
  }

  /**
   * Read the value of the X button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getXButton() {
    return getRawButton(Button.X_BUTTON.value);
  }

  /**
   * Whether the X button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getXButtonPressed() {
    return getRawButtonPressed(Button.X_BUTTON.value);
  }

  /**
   * Whether the X button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getXButtonReleased() {
    return getRawButtonReleased(Button.X_BUTTON.value);
  }

  /**
   * Read the value of the Y button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getYButton() {
    return getRawButton(Button.Y_BUTTON.value);
  }

  /**
   * Whether the Y button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getYButtonPressed() {
    return getRawButtonPressed(Button.Y_BUTTON.value);
  }

  /**
   * Whether the Y button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getYButtonReleased() {
    return getRawButtonReleased(Button.Y_BUTTON.value);
  }

  /**
   * Read the value of the back button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getBackButton() {
    return getRawButton(Button.BACK_BUTTON.value);
  }

  /**
   * Whether the back button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getBackButtonPressed() {
    return getRawButtonPressed(Button.BACK_BUTTON.value);
  }

  /**
   * Whether the back button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getBackButtonReleased() {
    return getRawButtonReleased(Button.BACK_BUTTON.value);
  }

  /**
   * Read the value of the start button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getStartButton() {
    return getRawButton(Button.START_BUTTON.value);
  }

  /**
   * Whether the start button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getStartButtonPressed() {
    return getRawButtonPressed(Button.START_BUTTON.value);
  }

  /**
   * Whether the start button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getStartButtonReleased() {
    return getRawButtonReleased(Button.START_BUTTON.value);
  }

  /**
   * Read the value of the left trigger (LT) button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftTrigger() {
    return getRawButton(Button.LEFT_TRIGGER.value);
  }

  /**
   * Read the value of the right trigger (RT) button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightTrigger() {
    return getRawButton(Button.RIGHT_TRIGGER.value);
  }

  /**
   * Whether the left trigger was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftTriggerPressed() {
    return getRawButtonPressed(Button.LEFT_TRIGGER.value);
  }

  /**
   * Whether the right trigger (RT) was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightTriggerPressed() {
    return getRawButtonPressed(Button.RIGHT_TRIGGER.value);
  }

  /**
   * Whether the left trigger (LT) was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftTriggerReleased() {
    return getRawButtonReleased(Button.LEFT_TRIGGER.value);
  }

  /**
   * Whether the right trigger (RT) was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightTriggerReleased() {
    return getRawButtonReleased(Button.RIGHT_TRIGGER.value);
  }
}
